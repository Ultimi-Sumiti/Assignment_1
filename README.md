# Assignment 1: The Turtlebot Navigation and Detection Pipeline

This repository contains the solution for "Assignment 1: The Turtlebot" for the course. The objective of this assignment is to develop a complete ROS 2 pipeline for the Turtlebot 3 to perform a complex task involving navigation, object detection, and sensor fusion within a simulated lab environment.

## 🚀 The Goal

The core mission for the Turtlebot 3 is to:
1.  **Navigate** to a specific position within the lab.
2.  The target position is **between two visible AprilTags**.
3.  From this target position, **detect three cylindrical tables** placed in the room.
4.  **Return the position of the tables** relative to the `odom` reference frame.

The robot spawns at the lab entrance at the launch of the simulation.

## 🛠️ Pipeline Components

The final pipeline, developed as a modular ROS 2 system, includes the following mandatory components:
* **AprilTag Detection:** Identifying the two AprilTags visible from a service camera. (Note: The AprilTag size is **0.050m x 0.050m**).
* **Navigation to AprilTags:** Calculating and reaching a safe position between the two detected AprilTags without hitting them.
* **Table Detection:** Detecting the three cylindrical tables using any available sensor (e.g., Lidar, camera, etc.).
* **Position Reporting:** Publishing the positions of the detected tables relative to the `odom` reference frame.

## ✨ Extra Points Implementation

*(If you implemented the extra points, include this section)*

We have implemented the extra credit challenge to demonstrate advanced navigation control:
* **Corridor Navigation:** Navigation to the final goal is **stopped** when the robot enters the corridor.
* **Custom Wall Following:** The robot switches to a **custom navigation method** inside the corridor, sending direct velocity commands and using the Lidar for wall detection.
* **Resumption:** Standard navigation to the AprilTag goal is **resumed** once the robot exits the corridor.

## 💻 How to Launch the Solution

This solution is designed to run within the pre-configured workspace from the assignment setup.

### 1. Prerequisites (If not done)

If you haven't already, please follow the initial setup steps to initialize your dedicated workspace and clone the necessary packages (`ir_2526`):

```bash
# Initialize a dedicated workspace
mkdir ws_$(group_number)_assignments
cd ws_$(group_number)_assignments
mkdir src
cd src
git clone [https://github.com/PieroSimonet/ir_2526.git](https://github.com/PieroSimonet/ir_2526.git)
cd ..
colcon build
source install/setup.bash
