# Represents a local goal for the robot's motion controller.
# Used primarily for path following and lane keeping.

# IMPORTANT: These values are derived from the 2D image space.
# The target is identified as a point in the image (e.g., the center of a line 
# or path), which is then projected to estimate its position relative 
# to the robot.

# The steering angle required to reach the target, typically in radians.
# Calculated based on the target's horizontal displacement in the visual field.
float64 steering_angle

# Estimated coordinates of the target point relative to the robot's base link.
# Note: These components are functional approximations based on image projection, 
# not high-precision spatial measurements.
# x_component: Estimated distance forward/backward.
# y_component: Estimated lateral distance (left/right).
float64 x_component
float64 y_component

float32 verticality # Orientation of the detected road component. 
                        # Used by the controller to filter false positives (e.g., zebra crossings) 
