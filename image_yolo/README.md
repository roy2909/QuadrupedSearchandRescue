## Overview
Human Detection and classification is done using the YOLOv8 model which is a realtime object detection and segmentation model. The model is trained on the COCO dataset and is able to detect and classify 80 different classes of objects. This was filtered to only detect human beings. Utilizing the depth information from the realsense camera, the Unitree Go1 is able to accurately and reliably determine the position (x,y,z coordinates and distance from the robot) of the human beings in the environment. This is visualized as red cubic markers in Rviz2.

## Dependencies
- [YOLOv8](https://yolov8.com/) - pip install ultralytics

## Launch files
`ros2 launch image_yolo image_launch.launch.xml use_rviz:=true`

