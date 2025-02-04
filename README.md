# JJCAR-Rescuebot
![alt text](Screenshot_from_2023-08-27_22-47-46.png "Search and Rescue")
A robot project that uses Spot the dog for search and rescue. For UTS 41068 - Robotics Studio 1.

## About
This ROS package gives Boston Dynamics' Spot the dog the capabilities for search and rescue. The main objective of this project is to have Spot search an area and identify a chosen target object.

## Installing the package
To install this package, you can directly clone the repo into your catkin_ws,

**If you have SSH setup, use that instead to not be prompted for authentication and have to use the PAT.**
```bash
cd ~/catkin_ws/src
git clone https://github.com/imJohly/JJCAR-Rescuebot.git
```
Then build the package as usual,
```bash
cd ~/catkin_ws
catkin_make
```

### Problem Statement
*"Search and rescue are a dangerous and costly task where emergency services need to quickly map out environments and localize rescue targets for a safe evacuation pathfinding. This means that rescue operations come with associated risks that need mitigation. Removing the need for humans to risk their lives would improve the overall process of search and rescue and minimise costs."*

**Project Objectives**
- Implementing Sensor Data and Spot API within ROS/Gazebo environment.
- Coding and Developing SLAM Algorithms and Pathfinding methods for Mapping and Localization. 
- Testing performance and efficiency of Search & Rescue method.  
- Live demonstration of Search & Rescue mission.

