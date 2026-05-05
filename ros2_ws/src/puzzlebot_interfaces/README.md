# puzzlebot_interfaces

This package defines the custom **ROS 2 message interfaces** that serve as the communication backbone for the Puzzlebot stack. These definitions standardize how perception data is structured before being consumed by the control layer.

---

## Message Definitions

The package provides three primary structures designed for high-level semantic communication:

---

### `NavigationTarget.msg`

Represents a specific waypoint or tracking point that the robot must follow.

- **Concept:** This is the core guidance for road-following; it represents a point derived from lane lines or road boundaries, acting as the robot's essential "guide" on the road.

- **`steering_angle`:** The calculated orientation required to reach the target.

- **`x_component` / `y_component`:** The relative coordinates of the target point in the robot's local frame.

- **`verticality`:**: Represents the angular orientation of the detected road component used to generate the navigation target. This parameter is essential for filtering false positives, ensuring path consistency when reacquiring the trajectory following maneuvers or intersections.

---

### `RoadPerception.msg`

Encapsulates the current state of the road infrastructure as seen by the camera.

- **`crosswalk_detected`:** Boolean flag to trigger pedestrian-zone logic or stopping behaviors.

- **`target_detected`:** Indicates if the vision pipeline has a valid road guide/target in view.

- **`target`:** A nested `NavigationTarget` message containing the steering and spatial data for the detected path.

- **`timestamp`:** Native ROS 2 time for synchronization.

---

### `TrafficSignDetection.msg`

The primary output for infrastructure recognition, combining raw detection with temporal persistence.

#### Functional Families (Categories)

Standardized mapping used by the tracker to ensure logical consistency:

| Constant        | Value | Description                                 |
| --------------- | ----- | ------------------------------------------- |
| `DIRECTION`     | 1     | Manoeuvre guidance (e.g., Turn Right/Left). |
| `TRAFFIC_LIGHT` | 2     | Signalized intersection control.            |
| `REGULATORY`    | 3     | High-priority priority signs (e.g., STOP).  |
| `CAUTION`       | 4     | Warnings or speed limit changes.            |

#### Fields

- **`sign_name`:** The specific class identified (e.g., `"stop"`, `"green_light"`).

- **`id`:** Unique persistent ID assigned by the tracker to distinguish between different physical signs.

- **`category`:** The functional family ID.

- **`timestamp`:** Time of capture for latency compensation.

---

## Data Flow & Usage

These interfaces allow for a strictly decoupled architecture across the workspace:

### Production (Publishers)

- **`road_perception` package**  
  Populates and publishes the `RoadPerception` message after analyzing the ground plane and road geometry.

- **`traffic_sign_tracker` package**  
  Populates and publishes the `TrafficSignDetection` message after filtering and associating YOLO outputs over time.

---

### Consumption (Subscribers)

- **`puzzlebot_controller` package**  
  Subscribes to both `RoadPerception` and `TrafficSignDetection`.  
  It acts as the primary consumer, using this semantic data to adjust linear and angular velocities within its FSM (Finite State Machine) logic.

---
