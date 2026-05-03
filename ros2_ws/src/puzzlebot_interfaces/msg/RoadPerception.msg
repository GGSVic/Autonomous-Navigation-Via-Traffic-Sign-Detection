# High-level road infrastructure and path tracking state.
# Consumed by the controller to manage state transitions (e.g., stopping at crosswalks).

# True if a crosswalk is identified in the current camera frame.
bool crosswalk_detected

# True if the road following algorithm has successfully locked onto a lane or path.
bool target_detected

# The specific navigation guide (point and angle) for the current path.
# Only valid if target_detected is True.
puzzlebot_interfaces/NavigationTarget target

# Standard ROS 2 timestamp for synchronization and latency monitoring.
builtin_interfaces/Time timestamp