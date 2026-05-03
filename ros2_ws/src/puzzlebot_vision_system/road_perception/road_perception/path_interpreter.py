"""
Path Interpreter Node.

Orchestrates the road perception pipeline by transforming raw camera streams into
semantic navigation targets. Handles image preprocessing, geometric interpretation
via SceneParser, and result broadcasting for the Puzzlebot control stack.

"""

import traceback
from typing import List
import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from cv_bridge import CvBridge
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Image

from rclpy.logging import LoggingSeverity
from road_perception.road_segmenter.road_component import RoadComponent
from road_perception.road_segmenter.road_segmenter import RoadSegmenter
from road_perception.utils.perception_context import PerceptionContext
from road_perception.utils.primitives import *
from road_perception.utils.road_visualizer import RoadVisualizer
from road_perception.scene.navigation_target import NavigationSet
from road_perception.scene.scene_parser import SceneParser

from puzzlebot_interfaces.msg import RoadPerception

BRIDGE = CvBridge()

class PathInterpreter(Node):
    """
    ROS 2 Node for road infrastructure interpretation and path planning.

    Subscribes to raw camera feeds and performs semantic segmentation to identify
    lanes and crosswalks. Publishes steering targets and detection flags via 
    the RoadPerception interface.
    """

    def __init__(self, node_name: str = "path_interpreter"):
        """
        Initializes the node, declares ROS parameters, and sets up communication.
        
        Args:
            node_name (str): Name of the node in the ROS graph.
        """
        super().__init__(node_name)
        
        self._declare_parameters()
        self._setup_communication()

        self.get_logger().info(f"{self.get_name()} successfully initialized!!")

    def _declare_parameters(self) -> None:
        """
        Declares ROS2 parameters to provide execution flexibility.
        """
        # Boolean debug parameter (default: False)
        self.declare_parameter(
            "debug",
            False,
            ParameterDescriptor(description="Enable/Disable visual debug and DEBUG logging"),
        )

        # Initial synchronization
        self.debug = self.get_parameter("debug").value
        self._update_logging_level()

    def _update_logging_level(self) -> None:
        """Updates logger severity based on the debug boolean."""
        level = LoggingSeverity.DEBUG if self.debug else LoggingSeverity.INFO
        self.get_logger().set_level(level)

    def _setup_communication(self):
        """
        Configures Publishers, Subscribers, and Quality of Service (QoS) profiles.
        """
        # Sensor Data QoS: Prioritizes real-time delivery over reliability.
        sensor_qos = QoSProfile(
            depth=1,
            reliability=qos_profile_sensor_data.reliability,
            durability=qos_profile_sensor_data.durability,
            history=qos_profile_sensor_data.history,
        )

        # Subscriber for the raw BGR8 camera stream.
        self.create_subscription(Image, "/camera/image_raw", self._image_callback, sensor_qos)
        
        # Publisher for annotated vision results (only active if debug:=True).
        self.output_img_pub = self.create_publisher(
            Image, "path_interpreter/processed_image", sensor_qos
        )

        # Publisher for the semantic state consumed by the Puzzlebot controller.
        self.results_pub = self.create_publisher(
            RoadPerception, "/path_interpreter/results", 10
        )

    def _image_callback(self, msg: Image) -> None:
        """
        Main pipeline callback triggered by new camera frames.

        Orchestrates the transition from raw pixels to semantic navigation roles 
        through ROI cropping, LAB-based segmentation, and geometric profiling.
        """
        # Convert ROS Image message to OpenCV BGR8 format.
        img_cv2 = BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # ROI selection: Analyze the bottom 50% of the frame (immediate road path).
        # Downsampling by 0.5 for optimized processing on embedded hardware.
        h_orig, w_orig = img_cv2.shape[:2]
        img_roi = img_cv2[int(h_orig / 2) :, :]
        img_roi = cv2.resize(img_roi, (0, 0), fx=0.5, fy=0.5, interpolation=cv2.INTER_LINEAR)

        # Global Perception Context initialization (Singleton-like state).
        h, w, _ = img_roi.shape
        if not PerceptionContext.is_initialized():
            # Ego-center: Robot position projected in the image plane (centered at the bottom).
            robot_position = Point2D(int(w / 2), h * 2)
            PerceptionContext.set_environment(w, h, robot_position)

        # ---- PERCEPTION PHASE ----
        # Isolates road lines and crosswalks using color segmentation and connectivity.
        percept: RoadSegmenter.RoadSegmentationOutput = RoadSegmenter.detect_road_components(
            img_roi
        )

        # ---- INTERPRETATION PHASE ----
        # Profiles road components to identify LEFT, RIGHT, or CENTER navigation targets.
        scene: SceneParser.SceneState = SceneParser.update_scene(
            percept.road_mask, percept.road_lines
        )

        # ---- TELEMETRY & OUTPUT ----
        if self.debug:
            self._visualize_results(img_roi, percept, scene)

        self._publish_results(scene.targets, percept.crosswalk)

    def _visualize_results(self, img: np.ndarray, percept: RoadSegmenter.RoadSegmentationOutput,
                          scene: SceneParser.SceneState) -> None:
        """
        Generates and publishes an annotated frame with the current perception state.
        
        Visualizes:
        - Detected road components (lines).
        - Crosswalk bounding boxes.
        - Lateral gradients and edge proximity analysis.
        - Final navigation targets and steering vectors.
        """
        debug_img = img.copy()
        
        RoadVisualizer.annotate_component(debug_img, percept.crosswalk)
        RoadVisualizer.render_components(debug_img, percept.road_lines)
        RoadVisualizer.render_analyses(debug_img, scene.analyses)
        RoadVisualizer.render_targets(debug_img, scene.targets.as_list)

        img_msg = BRIDGE.cv2_to_imgmsg(debug_img, encoding="bgr8")
        self.output_img_pub.publish(img_msg)

    def _publish_results(self, targets: NavigationSet, crosswalk: RoadComponent) -> None:
        """
        Serializes semantic data into the RoadPerception interface for broadcasting.
        
        Calculates the steering angle and its unit vector components (x, y) 
        to facilitate pure pursuit or PID control strategies.
        """
        msg = RoadPerception()
        msg.timestamp.sec, msg.timestamp.nanosec = self.get_clock().now().seconds_nanoseconds()

        # Binary detection flag for road infrastructure.
        msg.crosswalk_detected = crosswalk is not None

        # Lane guidance logic: Prioritizes the central target for path following.
        center_line = targets.center_line
        if center_line is not None:
            msg.target_detected = True
            msg.target.steering_angle = center_line.steering_angle
            
            # Unit vector projection relative to the robot's egocentric frame.
            msg.target.x_component = np.cos(center_line.steering_angle)
            msg.target.y_component = np.sin(center_line.steering_angle)
        else:
            msg.target_detected = False

        self.results_pub.publish(msg)

def main(args: List[str] = None) -> None:
    """
    Node entry point. Handles setup, spin-cycle, and clean shutdown.
    """
    rclpy.init(args=args)
    node = PathInterpreter()
    try:
        rclpy.spin(node)
    except Exception as error:
        traceback.print_exception(error)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    import sys
    main(sys.argv)