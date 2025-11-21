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
  
* **perception node:** Detecting the three cylindrical objects using lidar scan data, in particular we have converted the data from the topic into an cv::Mat object to then apply
  the Hough transform to remove all the straight line present in the new object.After these operations we performed again Hough, but this time to detect the circular objects in the 2d frame, this where the detections that where subsequently converted in the odom frame and sent into a dedicated topic.

<p align="center" >
  <img width="400" src="https://github.com/user-attachments/assets/be000230-d939-45f4-928a-1f463c6326f4" />
</p>


## Results:
### Result of nav2 navigation and apriltag detection
[Result of navigation and apriltag detection](https://github.com/user-attachments/assets/4f872ba5-8d36-44c5-ba33-10467093fbf6)

As you can see the robot correctly perform the posiotioning in the initial pose, passed through the topic \initialpose, also it correctly receive the goal position and plans
the  path using the costs matrix. In the end it detects the april tags and position itself in a close position as required.

### Results of cylinders detections
Detections of cylinders | Blurred intermediate file
--|--
![detections_result](https://github.com/user-attachments/assets/26cd9a31-7f16-4e80-b50f-ad39bb1cdd43)|![blur_result_detections](https://github.com/user-attachments/assets/eddab89d-b8a7-48f1-a715-14c7346d387e)

The left image contains some frames of the detection of the three cylinders, while the right frame contains the intermediate file used to perform the detections.



## How to launch

This solution is designed to run within the pre-configured workspace from the assignment setup.

### 1. Prerequisites (If not done)

If you haven't already, please follow the initial setup steps to initialize your dedicated workspace and clone the necessary packages (`ir_2526`):

```bash
#Clone the repository:
git clone [https://github.com/Ultimi-Sumiti/Assignment_1](https://github.com/Ultimi-Sumiti/Assignment_1)

#Position in the correct directory/workspace:
cd Assignment_1
cd ws_24_assignments

#Build the package:
colcon build

#Launch setup.bash: 
source install/setup.bash

#Launch the launch file:
ros2 launch nav_basics launch.py 
```


