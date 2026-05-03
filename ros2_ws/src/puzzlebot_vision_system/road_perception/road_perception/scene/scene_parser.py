"""
Geometric Scene Profiler and Feature Extractor.

Provides low-level geometric analysis of road components through cross-sectional
sampling and distance transforms. Computes spatial descriptors such as edge
proximity and lateral gradients to facilitate the classification of entities
based on their topological relationship with the road boundaries.
"""

from __future__ import annotations

import numpy as np
import cv2
from skimage.measure import profile_line

from dataclasses import dataclass
from typing import List, Tuple

from road_perception.road_segmenter.road_component import RoadComponent
from road_perception.utils.perception_context import PerceptionContext
from road_perception.scene.navigation_target import *


class SceneParser:

    @dataclass(frozen=True, slots=True)
    class EdgeProximity:
        """
        Describe the geometric relationship between a road component and the road boundaries.

        - distance_to_edge: Distance from the component to the nearest road boundary,
        computed along its perpendicular line.

        - lateral_gradient: Direction (-1.0 or 1.0) indicating toward which side
        the nearest boundary lies.
        """

        edge_proximity: int
        lateral_gradient: float

    @dataclass(frozen=True, slots=True)
    class ComponentAnalysis:
        """
        Data structure used during scene analysis to store
        derived metrics associated with a road component.

        - component: Reference to the corresponding RoadComponent.
        - proximity: Geometric relation to the road boundaries.
        """

        component: RoadComponent
        proximity: SceneParser.EdgeProximity

    @dataclass
    class SceneState:
        """
        Represents the interpreted state of the scene after perception.

        Combines intermediate analysis results with the final navigation
        targets used for decision-making.

        - analyses: Per-component geometric analysis derived from the scene.
        - targets: Structured navigation references (left, center, right)
        used to guide the robot.
        """

        analyses: List[SceneParser.ComponentAnalysis]
        targets: NavigationSet

    @staticmethod
    def update_scene(road_mask: np.ndarray, components: List[RoadComponent]) -> SceneState:
        """
        Orchestrates road component analysis and classification into a structured SceneState.

        Transforms raw perception inputs (road mask and components) into actionable
        navigation targets through geometric profiling and boundary-based classification.

        Args:
            road_mask: Binary image of detected road regions.
            components: List of identified RoadComponent instances.

        Returns:
            SceneState containing the geometric analysis and final navigation targets.
        """
        analyses: List[SceneParser.ComponentAnalysis] = SceneParser.analyze_components(
            road_mask, components
        )

        targets: NavigationSet = SceneParser.classify_targets(analyses)

        return SceneParser.SceneState(analyses, targets)

    @staticmethod
    def analyze_components(
        road_mask: np.ndarray, components: List[RoadComponent]
    ) -> List[ComponentAnalysis]:
        """
        Extracts cross-sectional profiles and computes geometric gradients for road components.

        Args:
            road_mask: Binary mask of the road elements.
            lines: List of RoadComponent objects to analyze.
        Returns:
            List of ComponentAnalysis containing gradient and safety data.
        """
        analysis_results: List[SceneParser.ComponentAnalysis] = []

        for comp in components:

            profile, idx = SceneParser._get_line_profile(road_mask, comp)

            dist_map, grad = SceneParser._analyze_geometric_profile(profile)

            lateral_gradient = float(grad[idx] * -1)

            distance = float(dist_map[idx])

            analysis_results.append(
                SceneParser.ComponentAnalysis(
                    comp, SceneParser.EdgeProximity(distance, lateral_gradient)
                )
            )

        return analysis_results

    @staticmethod
    def classify_targets(
        analyses: List[SceneParser.ComponentAnalysis], threshold: float = 30.0
    ) -> NavigationSet:
        """
        Classifies road components into navigation targets based on boundary proximity.

        Assignments are determined by geometric constraints:
        - Side lines: Components within 'threshold' are classified by 'lateral_gradient'
        direction (negative for LEFT, positive for RIGHT).
        - Center line: The candidate furthest from boundaries and closest to the frame's
        horizontal center is selected.

        Args:
            analyses: List of ComponentAnalysis with geometric metrics.
            threshold: Pixel distance to distinguish boundary vs. center candidates.

        Returns:
            NavigationSet with identified targets; fields are None if no matches occur.
        """
        output = NavigationSet()

        center_candidates: List[SceneParser.ComponentAnalysis] = []
        roi_center_x = PerceptionContext.frame_width() // 2

        for analysis in analyses:
            if analysis.proximity.edge_proximity < threshold:
                if analysis.proximity.lateral_gradient < 0:
                    output.left_line = NavigationTarget(
                        NavigationRole.LEFT_LINE, analysis.component.centroid
                    )
                else:
                    output.right_line = NavigationTarget(
                        NavigationRole.RIGHT_LINE, analysis.component.centroid
                    )
            else:
                center_candidates.append(analysis)

        if center_candidates:
            best_center = min(
                center_candidates, key=lambda e: abs(e.component.centroid.x - roi_center_x)
            )
            output.center_line = NavigationTarget(
                NavigationRole.CENTER_LINE, best_center.component.centroid
            )

        return output

    @staticmethod
    def _get_line_profile(
        road_mask: np.ndarray, component: RoadComponent
    ) -> Tuple[np.ndarray, int]:
        """
        Samples a 1D cross-sectional profile from a road mask using a component's perpendicular line.

        Computes the profile of pixel values and identifies the index (anchor)
        corresponding to the component's centroid for further geometric analysis.

        Args:
            road_mask: Binary road mask.
            component: RoadComponent with centroid and perpendicular line data.

        Returns:
            Tuple containing the 1D profile array and the centroid's index (anchor_idx).
        """

        # Extract geometry
        cx, cy = component.centroid.as_tuple
        perp_line = component.perp_line

        # Sample profile
        profile = profile_line(
            road_mask,
            perp_line.p_lefty.as_yx,
            perp_line.p_righty.as_yx,
            linewidth=1,
            mode="constant",
        )
        profile = np.array(profile)

        # Compute anchor index
        total_dist = np.hypot(
            perp_line.p_righty.x - perp_line.p_lefty.x,
            perp_line.p_righty.y - perp_line.p_lefty.y,
        )

        dist_to_anchor = np.hypot(
            cx - perp_line.p_lefty.x,
            cy - perp_line.p_lefty.y,
        )

        anchor_idx = int((dist_to_anchor / total_dist) * (len(profile) - 1))

        return profile, anchor_idx

    @staticmethod
    def _analyze_geometric_profile(line_profile: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
        """
        Transforms a 1D binary profile into a distance map and its spatial gradient.

        Uses a distance transform to measure proximity to the background and
        computes the gradient to determine the direction of the nearest edge.

        Args:
            line_profile: 1D binary array sampled from the road mask.

        Returns:
            Tuple containing the distance-to-edge map and the spatial gradient array.
        """

        # Compute the map of distances
        binary_samples = line_profile.astype(np.uint8)
        dist_map = cv2.distanceTransform(
            binary_samples.reshape(1, -1), cv2.DIST_L2, 5
        ).flatten()

        # Compute the gradient ot identify the direction to the closest 0 (background)
        grad = np.gradient(dist_map)

        return dist_map, grad
    