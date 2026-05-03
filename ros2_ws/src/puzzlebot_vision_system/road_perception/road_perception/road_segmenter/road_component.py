"""
Functional Road Geometry Abstraction.

Encapsulates OpenCV contours into high-level road entities. Provides lazy
evaluation of spatial features, including inertial fit-lines and perpendicular
profiling axes, enabling robust topological analysis of lanes, crosswalks,
and other navigational infrastructure.
"""

import numpy as np
import cv2
from enum import Enum

from road_perception.utils.primitives import *
from road_perception.utils.perception_context import PerceptionContext


class ComponentType(Enum):
    """
    Classification of road components used to dictate navigation behavior.
    
    Attributes:
        LINE: Representation of lane markings.
        CROSSWALK: Representation of pedestrian zones.
        LANE: Representation of the driveable surface area.
    """
    LINE = "line"
    CROSSWALK = "crosswalk"
    LANE = "lane"


class RoadComponent:
    """
    Represents a functional geometric element of the road.

    Validates integrity on initialization and uses lazy evaluation for
    computationally expensive geometric features.
    """

    def __init__(self, cnt: np.ndarray, label: ComponentType) -> None:
        """
        Initializes a RoadComponent from an OpenCV contour.

        Args:
            cnt: Input contour array representing the segmented object.
            label: The semantic classification of the component.
            
        Raises:
            ValueError: If the contour has no area (m00=0), preventing centroid calculation.
        """
        self._cnt: np.ndarray = cnt
        self._label: ComponentType = label
        self._fts: ContourFeatures = ContourFeatures()

        # Fail fast: Deterministic check for centroid availability
        moments = cv2.moments(self._cnt)
        if moments["m00"] == 0:
            raise ValueError("Invalid road component: contour has no area (m00=0).")

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])

        self._fts.centroid = Point2D(cx, cy)
        self._steering_angle: float | None = None

    @property
    def contour(self) -> np.ndarray:
        """Returns the raw OpenCV contour array."""
        return self._cnt

    @property
    def label(self) -> ComponentType:
        """Returns the semantic label (LINE, CROSSWALK, etc.)."""
        return self._label

    @property
    def centroid(self) -> Point2D:
        """Returns the geometric center of the component."""
        return self._fts.centroid

    @property
    def centroid_np(self) -> np.ndarray:
        """Returns the centroid as a NumPy array for vector operations."""
        return np.array([self._fts.centroid.x, self._fts.centroid.y])

    @property
    def area(self) -> float:
        """Lazily computes the pixel area of the contour."""
        if self._fts.area is None:
            self._fts.area = cv2.contourArea(self._cnt)
        return self._fts.area

    @property
    def bounding_box(self) -> BoundingBox:
        """Lazily computes the axis-aligned bounding box."""
        if self._fts.bbox is None:
            x, y, w, h = cv2.boundingRect(self._cnt)
            self._fts.bbox = BoundingBox(x, y, w, h)
        return self._fts.bbox

    @property
    def fitline(self) -> Line:
        """
        Lazily computes the inertial fit-line (least squares) of the contour.
        
        The line is extrapolated across the full width of the current PerceptionContext ROI.
        """
        if self._fts.fitline is None:
            # fitLine returns [vx, vy, x, y]
            [vx, vy, x, y] = cv2.fitLine(self._cnt, cv2.DIST_L2, 0, 0.01, 0.01)

            vx_val, vy_val = float(vx[0]), float(vy[0])
            slope = vy_val / vx_val if vx_val != 0 else 1e6

            # Compute verticality metric (0: Horizontal, 1: Vertical)
            angle_rad = np.arctan2(vy_val, vx_val)
            verticality = round(float(abs(np.sin(angle_rad))), 2)

            # Extrapolation using global PerceptionContext dimensions
            width = PerceptionContext.frame_width()
            lefty = int((-x * slope) + y)
            righty = int(((width - x) * slope) + y)

            self._fts.fitline = Line(
                Point2D(0, lefty), Point2D(width, righty), vx, vy, verticality
            )

        return self._fts.fitline

    @property
    def perp_line(self) -> Line:
        """
        Lazily computes a line perpendicular to the component's primary axis.
        
        Useful for profiling width or calculating lateral offsets from the centroid.
        """
        if self._fts.perp_line is None:
            # Invert fitline vectors to get perpendicular direction
            vx = -self.fitline.vy
            vy = self.fitline.vx

            cx, cy = self.centroid.as_tuple
            width = PerceptionContext.frame_width()
            height = PerceptionContext.frame_height()

            # Create extreme points for clipping
            raw_p1 = (int(cx + 2000 * vx), int(cy + 2000 * vy))
            raw_p2 = (int(cx - 2000 * vx), int(cy - 2000 * vy))

            # Clip line to stay within image boundaries
            _, p1, p2 = cv2.clipLine((0, 0, width, height), raw_p1, raw_p2)
            p_lefty, p_righty = (p1, p2) if p1[0] < p2[0] else (p2, p1)

            vx_val, vy_val = float(vx[0]), float(vy[0])
            angle_rad = np.arctan2(vy_val, vx_val)
            verticality = round(float(abs(np.sin(angle_rad))), 2)

            self._fts.perp_line = Line(
                Point2D(p_lefty[0], p_lefty[1]),
                Point2D(p_righty[0], p_righty[1]),
                vx,
                vy,
                verticality,
            )

        return self._fts.perp_line