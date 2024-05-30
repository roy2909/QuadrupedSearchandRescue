# Autonomous Exploration and Detection with a Unitree Go1 Robot

**Author: Rahul Roy**

## Overview

Rescue missions in disaster-stricken areas pose significant challenges due to their dangerous and unpredictable nature. The intricacies of these environments make it difficult to send humans for exploration, necessitating the utilization of ground robots or aerial robots. While conventional robots encounter limitations such as maneuvering through tight spaces and overcoming obstacles, quadrupeds offer promising solutions with their superior mobility.

To address this challenge, I have programmed a Unitree Go1 robot to autonomously explore unknown environments and search for survivors in disaster zones. Equipped with facial recognition capabilities, the robot can efficiently navigate through confined spaces and hazardous terrain, enhancing the effectiveness of rescue operations.

**Final Project Video:**

[![Project Video](https://github.com/roy2909/QuadrupedSearchandRescue/assets/144197977/d9c111cd-9b63-4f12-a80e-fc831fa61ddc)](https://github.com/roy2909/QuadrupedSearchandRescue/assets/144197977/d9c111cd-9b63-4f12-a80e-fc831fa61ddc)

**[Link to Portfolio Post](https://roy2909.github.io/Exploration/)**

## Dependencies

- [YOLOv8](https://yolov8.com/) - `pip install ultralytics`
- [DeepFace](https://pypi.org/project/deepface/) - `pip install deepface`

## Workspace Setup

1. **Setup your workspace:**
    ```sh
    mkdir -p ws/src
    ```

2. **Go into your source directory:**
    ```sh
    cd ws/src
    ```

3. **Download the `dependencies.repos` file:**
    ```sh
    wget <(https://github.com/roy2909/QuadrupedSearchandRescue/blob/main/dependencies.repos)>
    ```

4. **Use the VCS tool to import the dependencies:**
    ```sh
    vcs import < dependencies.repos
    ```

5. **Download the `rs_driver` submodule in `Lidar_ROS_2_SDK`:**
    ```sh
    cd rslidar_sdk_ros2/
    git submodule init
    git submodule update
    ```

6. **Build the workspace:**
    ```sh
    colcon build
    ```

7. **Source the workspace:**
    ```sh
    source install/setup.bash
    ```

## Launch Files

1. **Launch the Intel RealSense camera node:**
    ```sh
    ros2 launch realsense2_camera rs_launch.py enable_gyro:=true enable_accel:=true unite_imu_method:=1 enable_infra1:=true enable_infra2:=true enable_sync:=true
    ```

2. **Launch everything on the robot for mapping with the LiDAR only:**
    ```sh
    ros2 launch unitree_nav unitree_nav.launch.py
    ```

3. **Launch the robot with the RealSense camera and the LiDAR:**
    ```sh
    ros2 launch unitree_nav unitree_nav_camera.launch.py
    ```

4. **Launch the exploration node only:**
    ```sh
    ros2 launch frontier frontier.launch.xml
    ```

5. **Launch the exploration node and YOLO node for person detection:**
    ```sh
    ros2 launch frontier frontier_detect.launch.xml
    ```

Feel free to reach out if you have any questions or need further assistance with setting up the project.
