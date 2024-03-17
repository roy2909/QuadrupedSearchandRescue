## Overview
Rescue missions in disaster-stricken areas pose significant challenges due to their dangerous and unpredictable nature. The intricacies of these environments make it difficult to send humans for exploration, necessitating the utilization of ground robots or aerial robots. While conventional robots encounter limitations such as maneuvering through tight spaces and overcoming obstacles, quadrupeds offer promising solutions with their superior mobility.

To address this challenge, I have programmed a Unitree Go1 robot to autonomously explore unknown environments and search for survivors in disaster zones. Equipped with facial recognition capabilities, the robot can efficiently navigate through confined spaces and hazardous terrain, enhancing the effectiveness of rescue operations.

<iframe width="720" height="400" src="https://www.youtube.com/embed/zlveyBEczUs" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture" allowfullscreen></iframe>


 [Link to Portfolio Post](https://roy2909.github.io/Exploration/)


## Dependencies
- [YOLOv8](https://yolov8.com/) - pip install ultralytics
- [DeepFace](https://pypi.org/project/deepface/) - pip install deepface

## Workspace Setup

   1. Setup your workspace
   `mkdir -p ws/src`

   2. Go into your source directory
    `cd ws/src`

    3. Dowload the dependencies.repos file
    4. 
    5. Use the VCS tool to import the dependencies
   `vcs import < dependencies.repos`

    6. Now your workspace should have all the requires packages
   
    7. Lidar_ROS_2_SDK contains a submodule rs_driver that needs to be downloaded:
    `cd rslidar_sdk_ros2/`

   ` git submodule init`

    `git submodule update`

    8. Build the workspace
    `colcon build `
    
    9.  Source the workspace
   
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

   

