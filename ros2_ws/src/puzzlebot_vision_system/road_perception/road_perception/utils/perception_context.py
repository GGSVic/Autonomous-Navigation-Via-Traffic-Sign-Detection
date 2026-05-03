"""
Global Perception Environment Manager.

Synchronizes spatial constraints across the pipeline to maintain consistent
coordinate systems. This module prevents 'argument drilling' by providing
a centralized, thread-safe access point for the frame geometry and the 
robot's egocentric projection.
"""

from road_perception.utils.primitives import Point2D


class PerceptionContext:
    """
    State-persistent provider for global vision parameters.
    
    This class follows a singleton-style pattern to manage the spatial dimensions 
    of the current Region of Interest (ROI) and the robot's physical anchor point.
    """

    # Global ROI dimensions: Defaults to a 320x160 sub-frame
    _frame_width: int = 320
    _frame_height: int = 160

    # Egocentric reference: Projection of the Puzzlebot's footprint onto the image plane.
    # Default is the center-bottom of the frame.
    _robot_pos: Point2D = Point2D(160, 160)

    _initialized: bool = False

    @classmethod
    def set_environment(cls, frame_w: int, frame_h: int, robot_pos: Point2D) -> None:
        """
        Configures the spatial context for the entire perception stack.
        
        A 'write-once' policy is enforced to ensure coordinate consistency 
        throughout a single execution cycle.
        """
        if not cls._initialized:
            cls._frame_width = frame_w
            cls._frame_height = frame_h
            cls._robot_pos = robot_pos
            cls._initialized = True

    @classmethod
    def frame_width(cls) -> int:
        """Returns the width of the active perception ROI."""
        return cls._frame_width

    @classmethod
    def frame_height(cls) -> int:
        """Returns the height of the active perception ROI."""
        return cls._frame_height

    @classmethod
    def robot_pos(cls) -> Point2D:
        """Returns the Point2D anchor representing the robot's base-link in pixels."""
        return cls._robot_pos

    @classmethod
    def is_initialized(cls) -> bool:
        """Checks if the environment constraints have been defined."""
        return cls._initialized