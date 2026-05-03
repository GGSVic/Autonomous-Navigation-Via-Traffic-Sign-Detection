# Standardized constants for functional families. 
# These categories are used by the tracker to maintain semantic integrity 
# and by the FSM to prioritize high-level driving behaviors.

uint8 DIRECTION=1      # Guidance signs (e.g., arrows, turn signals)
uint8 TRAFFIC_LIGHT=2  # Infrastructure signals (Red, Green, Yellow)
uint8 REGULATORY=3     # High-priority rules (e.g., Stop, Yield)
uint8 CAUTION=4        # Warning/Infrastructure alerts (e.g., Speed limits)

# The human-readable class name from the YOLO model (e.g., "stop_sign", "red_light").
string sign_name

# The functional family ID. Used by the tracker as a strict matching constraint.
uint8 category

# Unique persistent identifier assigned by the Centroid Tracker.
# This ID remains consistent for the same physical object across frames.
int16 id

# ROS 2 timestamp for temporal synchronization with the perception stack.
builtin_interfaces/Time timestamp