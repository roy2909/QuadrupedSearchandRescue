## Facial Detection
The robot is equipped with facial recognition capabilities, allowing it to identify survivors in disaster zones. The robot uses the DeepFace library to detect and recognize faces. The robot can detect faces in real-time and compare them with a database of known faces to identify survivors. The robot can also store the faces of survivors in its database for future reference.

## Dependencies
- [DeepFace](https://pypi.org/project/deepface/) - pip install deepface

## Launch files
   `ros2 launch image_detection image_detection.launch.xml`

## Services
It has two services:
- `store` - store the faces of the person of interest
- `detect` - recognize the faces of the person of interest