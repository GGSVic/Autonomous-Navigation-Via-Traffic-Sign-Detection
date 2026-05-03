"""
Road Visualizer & Debug Overlays.

A specialized utility module for translating complex geometric abstractions, 
segmentation masks, and FSM navigation states into human-readable debug streams.
Essential for validating SceneParser logic and tracker performance.
"""

import cv2
import numpy as np
from typing import List

from road_perception.road_segmenter.road_component import RoadComponent, ComponentType
from road_perception.scene.scene_parser import SceneParser
from road_perception.utils.perception_context import PerceptionContext
from road_perception.scene.navigation_target import NavigationTarget, NavigationRole


class RoadVisualizer:
    """
    Static drawing engine for perception diagnostics.
    
    Provides high-level methods to render segmentation results, lateral gradient 
    vectors, and final navigation targets onto BGR image buffers.
    """

    # Semantic color mapping for road infrastructure
    COMPONENT_COLORS = {
        ComponentType.LINE: (255, 0, 0),       # Blue: Lane markings
        ComponentType.CROSSWALK: (0, 0, 255),   # Red: Stop/Zebra markings
        ComponentType.LANE: (0, 255, 0),       # Green: Driveable surface
    }

    # Semantic color mapping for navigation targets
    TARGET_COLORS = {
        NavigationRole.LEFT_LINE: (255, 100, 0),
        NavigationRole.CENTER_LINE: (0, 100, 255),
        NavigationRole.RIGHT_LINE: (0, 255, 100),
    }

    @staticmethod
    def annotate_component(img: np.ndarray, component: RoadComponent) -> None:
        """Renders a filled segmentation mask for a validated road component."""
        if component is None:
            return

        color = RoadVisualizer.COMPONENT_COLORS.get(component.label, (128, 128, 128))
        cv2.drawContours(img, [component.contour], -1, color, thickness=cv2.FILLED)

    @staticmethod
    def render_components(img: np.ndarray, components: List[RoadComponent]) -> None:
        """Batch renders a list of segmented components to the image buffer."""
        if not components:
            return
        for component in components:
            RoadVisualizer.annotate_component(img, component)

    @staticmethod
    def annotate_analysis(img: np.ndarray, analysis: SceneParser.ComponentAnalysis) -> None:
        """
        Visualizes the lateral gradient analysis for a specific component.
        
        Draws a diagnostic arrow indicating the direction of the road edge and 
        overlays a proximity label in pixels.
        """
        if analysis is None:
            return

        comp = analysis.component
        centroid = comp.centroid.as_tuple
        edge_proximity = analysis.proximity.edge_proximity
        grad_dir = analysis.proximity.lateral_gradient
        scan = comp.perp_line

        # Vector math to calculate the diagnostic arrow orientation
        vx_draw = scan.p_righty.x - scan.p_lefty.x
        vy_draw = scan.p_righty.y - scan.p_lefty.y
        scaled_vx = np.sign(vx_draw) * grad_dir * 10
        scaled_vy = np.sign(vy_draw) * grad_dir * 10
        end_point = (int(centroid[0] + scaled_vx), int(centroid[1] + scaled_vy))

        # Render gradient vector
        cv2.arrowedLine(img, centroid, end_point, (128, 0, 128), 2, tipLength=0.3)

        # Render proximity label (e.g., '45.2px')
        info = f"{edge_proximity:.1f}px" if edge_proximity < 100 else "-"
        FONT, SCALE, THICK = cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1
        (w, h), baseline = cv2.getTextSize(info, FONT, SCALE, THICK)
        text_pos = (end_point[0] + 8, end_point[1] + 4)

        # Contrast box for text readability
        cv2.rectangle(img, (text_pos[0] - 1, text_pos[1] - h - 1), 
                      (text_pos[0] + w + 1, text_pos[1] + baseline - 1), (0, 0, 0), -1)
        cv2.putText(img, info, text_pos, FONT, SCALE, (255, 255, 255), THICK, cv2.LINE_AA)

    @staticmethod
    def render_analyses(img: np.ndarray, analyses: List[SceneParser.ComponentAnalysis]) -> None:
        """Batch renders gradient analyses for all components in view."""
        if not analyses:
            return
        for analysis in analyses:
            RoadVisualizer.annotate_analysis(img, analysis)

    @staticmethod
    def annotate_target(img: np.ndarray, target: NavigationTarget) -> None:
        """
        Visualizes the final navigation target and steering path.
        
        Draws a line from the robot's egocentric center to the target point, 
        labeled with the navigation role and the calculated steering angle in degrees.
        """
        if target is None:
            return

        robot_pos = PerceptionContext.robot_pos()
        robot_coords = (int(robot_pos.x), int(robot_pos.y))
        pos = (int(target.centroid.x), int(target.centroid.y))
        color = RoadVisualizer.TARGET_COLORS.get(target.role, (128, 128, 128))

        # Render steering vector and target anchor
        cv2.circle(img, pos, 3, color, cv2.FILLED)
        cv2.circle(img, pos, 4, (255, 255, 255), 1)
        cv2.line(img, robot_coords, pos, color, 1, cv2.LINE_AA)

        # Format label text (e.g., 'center | 15.4deg')
        angle_deg = np.degrees(target.steering_angle)
        label_text = f"{target.role.value} | {angle_deg:.1f}deg"

        FONT, SCALE, THICK = cv2.FONT_HERSHEY_SIMPLEX, 0.4, 1
        (w, h), baseline = cv2.getTextSize(label_text, FONT, SCALE, THICK)
        box_y = pos[1] - 20

        # Background box for label
        cv2.rectangle(img, (pos[0], box_y - h - 5), 
                      (pos[0] + w + 4, box_y + baseline), (0, 0, 0), cv2.FILLED)
        cv2.putText(img, label_text, (pos[0] + 2, box_y), FONT, SCALE, (255, 255, 255), THICK, cv2.LINE_AA)

    @staticmethod
    def render_targets(img: np.ndarray, targets: List[NavigationTarget]) -> None:
        """Batch renders all active navigation targets."""
        if not targets:
            return
        for target in targets:
            RoadVisualizer.annotate_target(img, target)