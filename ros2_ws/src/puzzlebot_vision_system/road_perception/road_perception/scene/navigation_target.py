"""
Navigation Goal Abstraction and Spatial Targeting.

Defines the functional roles and persistent state of detected road elements
relative to the robot's egocentric frame. Provides a control-ready interface
by encapsulating the geometry required to compute steering angles and tracking
objectives from interpreted scene data.
"""

import numpy as np
from enum import Enum
from dataclasses import dataclass
from typing import List

from road_perception.utils.perception_context import PerceptionContext
from road_perception.utils.primitives import Point2D


class NavigationRole(Enum):
    """
    Functional categories for path-guiding elements.
    """
    LEFT_LINE = "left"
    CENTER_LINE = "center"
    RIGHT_LINE = "right"


@dataclass
class NavigationTarget:
    """
    Persistent navigation goal representing a refined identity.
    
    Stores the minimal spatial state required for high-frequency control loops
    and trajectory tracking.
    """

    role: NavigationRole
    centroid: Point2D

    @property
    def steering_angle(self) -> float:
        """
        Computes the target steering angle relative to the robot's projected origin.
        
        The calculation uses an egocentric reference frame provided by the 
        PerceptionContext.
        
        Returns:
            float: Steering angle in radians.
        """
        robot_pos = PerceptionContext.robot_pos()
        # Uses atan2 for robust quadrant-aware angle calculation
        # Normalized by -1.0 to align with standard robotics coordinate systems (left = positive)
        return float(
            -1.0 * np.arctan2(self.centroid.y - robot_pos.y, self.centroid.x - robot_pos.x)
        )


@dataclass(slots=True)
class NavigationSet:
    """
    Spatial container for the active navigation references in a single frame.

    Each target represents a high-level reference (Left, Center, Right) 
    used by the Finite State Machine (FSM) or PID controllers to guide 
    the robot's motion.
    """

    left_line: NavigationTarget | None = None
    center_line: NavigationTarget | None = None
    right_line: NavigationTarget | None = None

    @property
    def as_list(self) -> List[NavigationTarget]:
        """Returns all non-None targets for batch processing or visualization."""
        return [t for t in [self.left_line, self.center_line, self.right_line] if t is not None]