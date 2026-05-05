# Autonomous Navigation Via Traffic Sign Detection

## Overview

This repository presents the implementation and evolution of an autonomous mobile robotics stack designed for a scaled road environment. Originally conceived during my 6th-semester robotics implementation, this version represents a significant leap in algorithmic robustness, software architecture, and technical documentation.

The project features a Puzzlebot platform navigating a track with lanes/lines, intersections, and regulatory traffic signs. By fusing traditional computer vision techniques with deep learning (YOLOv8), the robot is capable of real-time semantic scene interpretation—allowing it to follow lines, respect traffic lights, and execute intelligent maneuvers at intersections without human intervention.

---

## Evolution of the Project

<p align="center">
  <img src="assets/original_demo.gif" width="720" alt="Legacy Implementation Demo"/>
</p>
<p align="center">
  <em>Early implementation of reactive line-following and basic computer vision on the Puzzlebot platform - 6th Semester Prototype (July 2024).</em>
</p>

This repository represents the architectural evolution of an autonomous navigation system originally developed in July 2024 during my 6th semester. While the core conceptual pipeline remains consistent, this iteration introduces a significant paradigm shift in modularity and algorithmic robustness. A key highlight is the sophisticated advancement of the computer vision stack, which has transitioned from a basic line-follower to a highly resilient perception engine capable of handling complex road scenarios with superior precision.

## Prerequisites & Environment

While the original project was successfully deployed on physical hardware (the Puzzlebot mobile platform), this repository is fully integrated with a high-fidelity simulation environment.

---

### System Requirements

- **Operating System:** Ubuntu 22.04 LTS (recommended)
- **Middleware:** ROS 2 Humble
- **Simulator:** Gazebo Fortress

---

### Hardware Acceleration

- **GPU (Recommended):** NVIDIA GPU with CUDA support for real-time `YOLOv8` inference

---

### Notes

- The system is designed for ROS 2 Humble and Gazebo Fortress
- The architecture is modular and can be adapted to other ROS 2 or Gazebo distributions with minor adjustments

---

## Deployment Guide: Puzzlebot Autonomous Stack

This guide provides the necessary steps to set up the Puzzlebot simulation and navigation environment using Docker. It ensures a deterministic environment, avoiding local dependency conflicts.

> **Alternative (Local Execution):**  
> If your system already satisfies the required dependencies (ROS 2 Humble, Gazebo Fortress, CUDA support, etc.), you may run the project natively without Docker.  
> In this case, you can skip directly to **Step 3 (Build the Workspaces)**.  
> Note that this approach may introduce inconsistencies depending on your local setup.

---

### 1. Clone the Repository

To ensure all components, including controllers and simulation assets, are present, clone the repository with its submodules:

```bash
git clone --recurse-submodules https://github.com/GGSVic/Autonomous-Navigation-Via-Traffic-Sign-Detection.git puzzlebot_nav_stack
cd puzzlebot_nav_stack
```

---

### 2. Container Configuration (Docker)

#### a. GPU Support (Optional)

If you have NVIDIA hardware, it is highly recommended to use the NVIDIA Container Toolkit. This accelerates `YOLOv8` inference and Ignition Gazebo rendering.

For installation instructions, refer to the official NVIDIA guide:

- [NVIDIA Container Toolkit Installation Guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)

---

#### Validate GPU Support

You can verify that Docker has access to your GPU by running the following command:

```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

**Expected Result:**

If everything is correctly configured, you should see a table showing your GPU information (name, driver version, CUDA version, memory usage, etc.).

- If you see GPU details → **CUDA is correctly configured**
- If the command fails or shows no devices → GPU support is not properly set up

---

#### b. Build the Image

From the project root, build the custom image using the development container configuration:

```bash
docker build -t puzzlebot_nav_stack:latest -f .devcontainer/Dockerfile .devcontainer/

```

---

#### c. Run the Container

Launch the container with GUI support and volume mounting.

**With GPU Support:**

```bash
docker run -it --rm --name puzzlebot_instance \
    --gpus all --net=host --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PWD:/home/ubuntu/project" \
    puzzlebot_nav_stack:latest
```

**Without GPU (Standard Mode):**

```bash
docker run -it --rm --name puzzlebot_instance \
    --net=host --privileged \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$PWD:/home/ubuntu/project" \
    puzzlebot_nav_stack:latest
```

> A Terminator window will open automatically if the setup is correct.

---

### 3. Build the Workspaces

Inside the container (or locally, if running without Docker), navigate to the project root to compile the ROS 2 workspaces.

#### Simulation Workspace (Ignition Gazebo)

Before moving on to the compilation, make sure to update your system repositories so rosdep can work correctly:

```bash
sudo apt update
```

Includes the Puzzlebot models and the autonomous navigation track.

```bash
cd /home/ubuntu/project/simulation_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source install/setup.bash
cd ..
```

---

#### Navigation Workspace (ROS 2)

Contains the FSM, PID controllers, and vision nodes.

```bash
cd ros2_ws
rosdep install -i --from-path src --rosdistro humble -y
colcon build
source install/setup.bash
```

---

### 4. Run the System

Before launching, you must set the environment variable to specify the correct robot model. Then, launch the master script to start the simulation and control nodes.

```bash
export PUZZLEBOT_MODEL=vision
ros2 launch puzzlebot_controller master.launch.py
```

---

### Available Arguments

- `debug:=true`  
  Enables visualization windows for YOLOv8 and line detection results.

- `device_type:=cuda`  
  Forces the vision algorithms to run on CUDA cores.

- `world:='puzzletrack_v2.sdf'`
  You can choose a different world to test the project.

---

## Expected Results

Below is a demonstration of the autonomous navigation stack in action. The GIF showcases the robot's ability to navigate complex intersections, obey traffic laws, and utilize its new perceptual-based recovery logic to resume line following after maneuvers.

<p align="center">
  <img src="assets/output_demo.gif" width="720" alt="Autonomous Navigation Demo"/>
</p>

<p align="center">
  <em>Autonomous navigation with perception-based recovery and traffic-aware decision making.</em>
</p>

> **Note:** The video footage was captured from a camera temporarily mounted behind the robot for demonstration purposes; this specific camera configuration is not included in the standard `ros_gz_puzzlebot_description` package.  
> The demonstration GIF is displayed at **2× speed** for brevity.

---

For a deep dive into the system's performance and architecture, you can check out the following resources:

- **[Full System Overview](https://youtu.be/CI7L01OlaB0):** A brief explanatory video covering the system architecture, road morphology logic, and implementation details.
- **[Autonomous Navigation Demo](https://youtu.be/39QUw0jZBZ0):** A detailed demonstration of the robot's performance in the simulation environment.

---

## Project Structure

The repository is divided into two main workspaces to separate the simulation environment from the core autonomous logic. This decoupling ensures modularity and simplifies deployment across different hardware configurations.

---

### Simulation Workspace (`simulation_ws`)

This workspace handles the digital twin of the road environment and the physical modeling of the robot platform.

- **`ros_gz_plugins/traffic_light_plugin`** _(Git Submodule)_  
  A specialized Gazebo plugin used to simulate realistic traffic light behaviors and states.

- **`ros_gz_puzzlebot/ros_gz_puzzlebot_description`** _(Git Submodule)_  
  Contains the URDF/Xacro files, meshes, and physical parameters of the Puzzlebot.

- **`ros_gz_puzzlebot/ros_gz_puzzlebot_bringup`** _(Git Submodule)_  
  Launch files used to spawn the robot and load the road world within the Gazebo environment.

---

### Autonomous Logic Workspace (`ros2_ws`)

This is the core of the project, containing the intelligence, perception, and control nodes.

- **`puzzlebot_controller`**  
  The decision-making heart. It uses a Multi-threaded FSM to manage high-level behaviors and a PID controller for precise movement. It is now optimized to exit maneuvers based on path-recovery events.

- **`puzzlebot_interfaces`**  
  Custom ROS 2 message and service definitions for semantic communication between nodes.

- **`road_perception`**  
  Handles lane segmentation and semantic scene parsing using color filtering and lateral gradient analysis.

- **`traffic_sign_tracker`**  
  A YOLOv8-based node that identifies and tracks traffic signs and lights to update the robot's internal world-view.

---

> **Tip:** Each package contains its own detailed documentation. To dive deeper into the specific implementation of the algorithms, please visit the individual README files within each folder.

---
