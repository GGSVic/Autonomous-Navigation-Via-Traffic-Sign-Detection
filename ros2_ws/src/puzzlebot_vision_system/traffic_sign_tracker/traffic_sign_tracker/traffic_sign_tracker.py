#!/usr/bin/env python3
"""
Traffic Sign Detection and Tracking Node.

This node integrates a YOLO-based inference engine with a centroid tracking system.
It processes raw camera feeds to identify, categorize, and track road infrastructure
elements, publishing high-level semantic data for the robot's control stack.
"""

import os
import traceback
from typing import List, Optional

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.logging import LoggingSeverity
from rcl_interfaces.msg import ParameterDescriptor

from sensor_msgs.msg import Image
from ultralytics import YOLO  # type: ignore
from ament_index_python import get_package_share_directory

from puzzlebot_interfaces.msg import TrafficSignDetection
from traffic_sign_tracker.tracker.detection import Detection
from traffic_sign_tracker.tracker.track import Track
from traffic_sign_tracker.tracker.centroid_tracker import CentroidTracker
from traffic_sign_tracker.utils.visualizer import TrafficSignVisualizer

BRIDGE = CvBridge()


class TrafficSignTracker(Node):
    """
    ROS2 Node for real-time traffic sign perception.

    Orchestrates the computer vision pipeline: image acquisition, YOLO inference,
    temporal tracking, and semantic results broadcasting.
    """

    def __init__(self, node_name: str = "traffic_sign_tracker") -> None:
        """
        Initializes the perception pipeline, hardware configurations, and ROS2 
        communication interfaces.
        """
        super().__init__(node_name)

        self._declare_parameters()
        self._set_auxiliar_variables()
        self._setup_models()
        self._setup_communication()

        self.get_logger().info(f"{self.get_name()} successfully initialized!!!")

    def _declare_parameters(self) -> None:
        """
        Declares ROS2 parameters to provide execution flexibility.
        """
        self.declare_parameter(
            "device_type",
            "cpu",
            ParameterDescriptor(description="Hardware accelerator for inference ('cpu', 'cuda')"),
        )
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

    def _setup_models(self) -> None:
        """
        Loads YOLO weights from the package's share directory and configures 
        the inference engine on the specified hardware device.
        """
        device_type = self.get_parameter("device_type").get_parameter_value().string_value

        share_directory = get_package_share_directory("traffic_sign_tracker")
        path_to_model = os.path.join(share_directory, "models", "best.pt")

        if not os.path.exists(path_to_model):
            self.get_logger().fatal(f"Model not found in: {path_to_model}")
            raise FileNotFoundError(f"Missing YOLO weights: {path_to_model}")
        try:
            self.model = YOLO(path_to_model)
            self.model.to(device_type)
            self.get_logger().info(f"YOLO Inference engine started on {device_type.upper()}")
        except Exception as e:
            self.get_logger().fatal(f"Failed to load YOLO model: {str(e)}")
            raise e

    def _setup_communication(self) -> None:
        """
        Configures publishers, subscribers, and Quality of Service (QoS) profiles 
        for efficient image and data transport.
        """
        qos_profile = qos_profile_sensor_data
        qos_profile.depth = 1

        self.create_subscription(Image, "camera/image_raw", self._image_callback, qos_profile)

        self.debug_pub = self.create_publisher(
            Image, "traffic_sign_tracker/processed_image", qos_profile
        )
        self.results_pub = self.create_publisher(
            TrafficSignDetection, "traffic_sign_tracker/results", 10
        )

        self.tsd_msg = TrafficSignDetection()

    def _set_auxiliar_variables(self) -> None:
        """
        Initializes non-ROS state variables, including the temporal tracking engine.
        """
        self.tracker = CentroidTracker()

    def _image_callback(self, msg: Image) -> None:
        """
        Core perception pipeline execution triggered by new camera frames.

        Process flow:
        1. Image Conversion
        2. Inference (YOLO)
        3. Tracking (CentroidTracker)
        4. Debug Visualization (Only if debug=True)
        5. Data Publishing
        """
        img_cv2 = BRIDGE.imgmsg_to_cv2(msg, desired_encoding="bgr8")

        # Inference
        results = self.model.predict(img_cv2, verbose=False, imgsz=640, iou=0.45)

        # Tracking
        detections = Detection.from_yolo_results(results, self.model.names)
        tracks = self.tracker.update(detections)

        # Visual Debug (Controlled by the manual boolean)
        if self.debug:
            TrafficSignVisualizer.render_tracks(img_cv2, tracks)
            debug_msg = BRIDGE.cv2_to_imgmsg(img_cv2, encoding="bgr8")
            self.debug_pub.publish(debug_msg)

        # Results publishing
        self._publish_results(tracks)

    def _publish_results(self, tracks: List[Track]) -> None:
        """
        Populates and broadcasts the custom TrafficSignDetection message.
        """
        sec, nanosec = self.get_clock().now().seconds_nanoseconds()
        self.tsd_msg.timestamp.sec = sec
        self.tsd_msg.timestamp.nanosec = nanosec

        for track in tracks:
            # Consistent with track.latest_detection
            self.tsd_msg.sign_name = track.latest_detection.name
            self.tsd_msg.category = track.latest_detection.category
            self.tsd_msg.id = track.id
            self.results_pub.publish(self.tsd_msg)


def main(args: Optional[List[str]] = None) -> None:
    """
    Node entry point.
    """
    rclpy.init(args=args)
    node = TrafficSignTracker()
    
    try:
        rclpy.spin(node)
    except Exception as error:
        traceback.print_exception(error)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()