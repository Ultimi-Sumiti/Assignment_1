# Assignment 1: The Turtlebot Navigation and Detection Pipeline

## Description:

This repository contains the solution for "Assignment 1: The Turtlebot" for the course of Intelligent Robotics. The objective of this assignment is to develop a complete ROS 2 pipeline for the Turtlebot 3 to perform a complex task involving navigation, object detection, and sensor data transformation within a simulated enviroment.

## Goal:

The core mission for the Turtlebot 3 is to:
1.  **Navigate** to a specific position within the simulate enviroment (using the nav2 framework).
2.  The target position is **between two visible AprilTags**.
3.  From this target position, **detect three cylindrical tables** placed in the room.
4.  **Return the position of the cylinders centers** relative to the `odom` reference frame.

(The robot spawns at the lab entrance at the launch of the simulation)

## Pipeline Components:

The final pipeline, developed as a modular ROS 2 package, includes the following components/nodes:
* **lifecycle node client:** This node is the client which interact with the nav2 navigation framework, sending initial position and request to the localization and navigation components.
* **navigation action client:** This node is an action client sending goal request to the nav2 framework and waiting for response and feedback, to then calculate and reach a safe position between the two detected AprilTags without hitting them.
* **Cyk Detection:** Detecting the three cylindrical tables using any available sensor (e.g., Lidar, camera, etc.).
* **Position Reporting:** Publishing the positions of the detected tables relative to the `odom` reference frame.

*

We have implemented the extra credit challenge to demonstrate advanced navigation control:
* **Corridor Navigation:** Navigation to the final goal is **stopped** when the robot enters the corridor.
* **Custom Wall Following:** The robot switches to a **custom navigation method** inside the corridor, sending direct velocity commands and using the Lidar for wall detection.
* **Resumption:** Standard navigation to the AprilTag goal is **resumed** once the robot exits the corridor.

## How to launch

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

https://github.com/user-attachments/assets/4f872ba5-8d36-44c5-ba33-10467093fbf6
