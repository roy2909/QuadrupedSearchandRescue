# Autonomous Exploration and Detection with A Unitree Go1 Robot

## Overview
Rescue missions in disaster-stricken areas pose significant challenges due to their dangerous and unpredictable nature. The intricacies of these environments make it difficult to send humans for exploration, necessitating the utilization of ground robots or aerial robots. While conventional robots encounter limitations such as maneuvering through tight spaces and overcoming obstacles, quadrupeds offer promising solutions with their superior mobility.

To address this challenge, I have programmed a Unitree Go1 robot to autonomously explore unknown environments and search for survivors in disaster zones. Equipped with facial recognition capabilities, the robot can efficiently navigate through confined spaces and hazardous terrain, enhancing the effectiveness of rescue operations.

Final Project Video: 

https://github.com/roy2909/QuadrupedSearchandRescue/assets/144197977/d9c111cd-9b63-4f12-a80e-fc831fa61ddc


 [Link to Portfolio Post](https://roy2909.github.io/Exploration/)


## Dependencies
- [YOLOv8](https://yolov8.com/) - pip install ultralytics
- [DeepFace](https://pypi.org/project/deepface/) - pip install deepface

## Workspace Setup

   1. Setup your workspace
      `mkdir -p ws/src`

   2. Go into your source directory
       `cd ws/src`

    3. Download the dependencies.repos file

    4. Use the VCS tool to import the dependencies
      `vcs import < dependencies.repos`

    5. Now your workspace should have all the requires packages
   
    6. Lidar_ROS_2_SDK contains a submodule rs_driver that needs to be downloaded:
   
    `cd rslidar_sdk_ros2/`

    `git submodule init`

    `git submodule update`

    7. Build the workspace
     `colcon build `

    8. Source the workspace
   
## Launch files
1. Launch the intel realsense camera node
   `ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true`
2. Use the to launch everything on the robot if you want mapping with the lidar only
    `ros2 launch unitree_nav unitree_nav.launch.py`

3. Use the following command to launch the robot with the realsense camera and the lidar
   `ros2 launch unitree_nav unitree_nav_camera.launch.py`

4. Use the following command to launch exploration node only
   `ros2 launch frontier frontier.launch.xml`

5. Use the following command to launch exploration node and yolo node for person detection
   `ros2 launch frontier frontier_detect.launch.xml`

   

