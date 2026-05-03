# Autonomous Navigation Via Traffic Sign Detection
##  Overview

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

Operating System: Ubuntu 22.04 LTS (recommended).

Middleware: ROS 2 Humble Hawksbill.

Simulator: Gazebo Fortress.

Hardware Acceleration: NVIDIA GPU with CUDA support (highly recommended for real-time YOLOv8 inference).

Note: While designed for Humble and Fortress, the logic and algorithms are modular enough to be adapted to other ROS 2 or Gazebo distributions with minor adjustments.

--- 

## How to Run