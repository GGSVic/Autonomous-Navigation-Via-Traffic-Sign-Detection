# traffic_sign_tracker

This package manages the detection and tracking of traffic infrastructure for the Puzzlebot. It transforms raw camera frames into stable, categorized data, ensuring the robot perceives its environment consistently over time rather than relying on flickering, instantaneous glimpses.

---

## Overview

For autonomous navigation, a robot must "remember" what it sees. This package provides a **Temporal Memory system** that solves common vision challenges:

- **Custom Tracking:** Unlike standard YOLO trackers, this implementation is optimized for robotics. It is lightweight, allows for high execution efficiency, and permits deep customization of how objects are identified.

- **Persistent Identity:** Ensures that a sign detected in one frame is recognized as the same physical object in subsequent frames.

- **Logical Grouping:** Prevents `ID swapping` by forcing detections to stay within their functional families (e.g., a "Stop" sign will never be confused with a "Traffic Light").

---

<p align="center">
  <img src="assets/header_demo.gif" width="320" alt="Traffic Sign Tracker Overview"/>
</p>
<p align="center">
  <em>Real-time traffic sign tracking mounted on the Puzzlebot platform.</em>
</p>

---

## Perception Pipeline

The system processes visual information through a modular sequence:

1. **Object Detection:** Identifying raw bounding boxes and YOLO classes.

2. **Local Data Conversion:** Translating YOLO outputs into categorized internal objects.

3. **Tracking Implementation:** Associating detections with existing tracks over time.

4. **Cleaning & Refinement:** Removing stale tracks and smoothing data.

5. **Results Publishing:** The node continuously publishes all active tracks. The end consumer must be designed to handle persistent data streams appropriately to avoid redundant processing of the same entity.

---

## Note on TF Locking

Detecting traffic lights from various angles is a significant challenge. Because the model was initially trained to detect lights from multiple perspectives, it introduces noise and makes it difficult to identify which specific signal governs the current path.

To address this, the tracker implements **`TF Locking`**. Since the valid traffic light is typically detected much earlier and more clearly than non-relevant ones, the system "locks" onto it and maintains focus until it leaves the field of view. While advanced systems might use stereo vision, this approach provides a reliable and efficient solution for the project's specific constraints.

---

## File Structure & Data Flow

The system is architected as a streamlined process to refine raw pixels into high-level intelligence:

---

### Core Logic

**`tracker/detection.py` (Semantic Filtering)**  
Translates raw outputs into a specialized `Detection` class. It assigns Functional Families (`Direction`, `Regulatory`, `Traffic Light`, `Caution`), which forces the tracking algorithm to be more precise and prevents ID swaps between different infrastructure types.

**`tracker/track.py` (Temporal Memory)**  
Defines the `Track` object. Each track represents a unique physical entity. It manages the life cycle—tracking `age` and `missed frames`—allowing the robot to "remember" a sign even if it is temporarily occluded.

**`tracker/centroid_tracker.py` (The Association Engine)**  
The heart of the package. It uses a custom implementation to solve the `assignment problem`:

- **Matching:** Links detections to existing tracks based on distance and category.  
- **Innovation:** Initializes new tracks for infrastructure discovered as the robot moves.  
- **TF Locking:** Specifically manages intersection noise by locking onto the primary traffic light and ignoring background objects.

---

### Orchestration & Support

**`traffic_sign_tracker.py` (The Orchestrator Node)**  
The ROS 2 entry point. It manages image subscriptions, coordinates the inference loop, and handles dynamic parameters for debugging.

**`utils/visualizer.py` (Custom Rendering)**  
Since the tracker uses a custom data structure instead of raw YOLO results, this module provides an OpenCV-based renderer to draw history, unique IDs, and family-specific colors for active tracks.

---

## ROS 2 Interface

### Parameters

| Parameter      | Type   | Description |
|----------------|--------|------------|
| `device_type`  | string | Selects the inference hardware (e.g., cpu or cuda) |
| `debug`        | bool   | When True, it enables debug logging and publishes the processed_image stream |

---

### Topics

| Topic                                      | Type          | Description |
|-------------------------------------------|--------------|------------|
| `camera/image_raw`                         | Subscription | Raw BGR input |
| `traffic_sign_tracker/results`             | Publisher    | Semantic data (`TrafficSignDetection`) for the control stack |
| `traffic_sign_tracker/processed_image`     | Publisher    | Visual telemetry for debugging (only active in debug mode) |

---

## How to Run

To launch the tracker with visual debugging enabled:

```bash
ros2 run traffic_sign_tracker tracker --ros-args -p debug:=True
```

`Note:` The debug parameter defaults to False for optimal production performance.

---