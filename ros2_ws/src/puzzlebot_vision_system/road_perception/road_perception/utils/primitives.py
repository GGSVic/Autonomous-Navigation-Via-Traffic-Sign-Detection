"""
Primitive Data Structures for Robotic Vision.

Lightweight, immutable containers for geometric entities and visual properties.
Designed for high-frequency access with minimal memory footprint.
"""

from dataclasses import dataclass
import numpy as np

@dataclass(frozen=True, slots=True)
class Point2D:
    """Represents a discrete 2D coordinate in the image coordinate system."""
    x: int
    y: int

    @property
    def as_tuple(self) -> tuple[int, int]:
        """Returns coordinates as (x, y) for OpenCV functions."""
        return (self.x, self.y)

    @property
    def as_yx(self) -> tuple[int, int]:
        """Returns coordinates as (y, x) for NumPy/SciKit-Image matrix indexing."""
        return (self.y, self.x)


@dataclass(frozen=True, slots=True)
class BoundingBox:
    """Immutable axis-aligned rectangular region defined by (x, y, width, height)."""
    x: int
    y: int
    w: int
    h: int


@dataclass(frozen=True, slots=True)
class Line:
    """
    Immutable representation of a geometric line segment.
    
    Attributes:
        p_lefty: The point where the line intersects the left ROI boundary.
        p_righty: The point where the line intersects the right ROI boundary.
        vx, vy: Unit vector components of the line's orientation.
        verticality: A metric (0.0 to 1.0) indicating how 'upright' the line is.
    """
    p_lefty: Point2D
    p_righty: Point2D
    vx: np.ndarray
    vy: np.ndarray
    verticality: float


@dataclass(slots=True)
class ContourFeatures:
    """
    Cache for spatial descriptors derived from OpenCV contours.
    
    This class is mutable to support the 'Lazy Evaluation' pattern used 
    in RoadComponent, allowing features to be computed only when requested.
    """
    area: float | None = None
    centroid: Point2D | None = None
    bbox: BoundingBox | None = None
    fitline: Line | None = None
    perp_line: Line | None = None


@dataclass(frozen=True, slots=True)
class Color:
    """Immutable RGB representation for consistent annotation styles."""
    r: int
    g: int
    b: int