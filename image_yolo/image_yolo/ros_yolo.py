#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
# import pyrealsense2 as rs2
# if (not hasattr(rs2, 'intrinsics')):
#     import pyrealsense2.pyrealsense2 as rs2



from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class Camera_subscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        # self.sub1 = self.create_subscription(msg_Image, '/camera/depth/image_rect_raw', self.imageDepthCallback, 1)
        # self.sub_info = self.create_subscription(CameraInfo, '/camera/depth/camera_info', self.imageDepthInfoCallback, 1)
        self.sub = self.create_subscription(msg_Image, '/camera/color/image_raw', self.camera_callback, 1)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        self.model = YOLO('/home/rahulroy/winter_project_ws/src/unitree/image_yolo/image_yolo/yolov8n.pt')
        self.person_centroids = []
        self.yolov8_inference = Yolov8Inference()


        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(msg_Image, "/inference_result", 1)


    def detect_human(self, image):
        results = self.model(image)
        for r in results:
            boxes = r.boxes
            for box in boxes:
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                centroid = ((b[1] + b[3]) // 2, (b[0] + b[2]) // 2)
                self.person_centroids.append(centroid)
        return self.person_centroids


    def camera_callback(self, data):
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)

        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        for r in results:
            boxes = r.boxes
            for box in boxes:
                c = box.cls
                class_name = self.model.names[int(c)]
                print(class_name)
                
                if class_name == 'person':
                    self.inference_result = InferenceResult()
                    b = box.xyxy[0].to('cpu').detach().numpy().copy() 
                    # get box coordinates in (top, left, bottom, right) format
                    self.inference_result.class_name = class_name
                    self.inference_result.top = int(b[0])
                    self.inference_result.left = int(b[1])
                    self.inference_result.bottom = int(b[2])
                    self.inference_result.right = int(b[3])
                    self.yolov8_inference.yolov8_inference.append(self.inference_result)

        # Draw bounding box only if the class name is "person"
        if len(self.yolov8_inference.yolov8_inference) > 0:
            annotated_frame = results[0].plot()
            img_with_overlay = cv2.addWeighted(img, 0.7, annotated_frame, 0.3, 0)
            img_msg = bridge.cv2_to_imgmsg(img_with_overlay)
            img_msg = bridge.cv2_to_imgmsg(annotated_frame)  

            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

        
        # self.pin_centroids = self.detect_pins(img)
    
    
        
   

    # def imageDepthCallback(self, data):
    #     try:
    #         cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    #         # pick one pixel among all the pixels with the closest range:
    #         indices = np.array(np.where(cv_image == cv_image[cv_image > 0].min()))[:,0]
    #         pix = (indices[1], indices[0])
    #         self.pix = pix
    #         line = '\rDepth at pixel(%3d, %3d): %7.1f(mm).' % (pix[0], pix[1], cv_image[pix[1], pix[0]])

    #         if self.intrinsics:
    #             depth = cv_image[pix[1], pix[0]]
    #             result = rs2.rs2_deproject_pixel_to_point(self.intrinsics, [pix[0], pix[1]], depth)
    #             line += '  Coordinate: %8.2f %8.2f %8.2f.' % (result[0], result[1], result[2])
    #         if (not self.pix_grade is None):
    #             line += ' Grade: %2d' % self.pix_grade
    #         line += '\r'
    #         sys.stdout.write(line)
    #         sys.stdout.flush()

    #     except CvBridgeError as e:
    #         print(e)
    #         return
    #     except ValueError as e:
    #         return


    # def imageDepthInfoCallback(self, cameraInfo):
    #     try:
    #         if self.intrinsics:
    #             return
    #         self.intrinsics = rs2.intrinsics()
    #         self.intrinsics.width = cameraInfo.width
    #         self.intrinsics.height = cameraInfo.height
    #         self.intrinsics.ppx = cameraInfo.k[2]
    #         self.intrinsics.ppy = cameraInfo.k[5]
    #         self.intrinsics.fx = cameraInfo.k[0]
    #         self.intrinsics.fy = cameraInfo.k[4]
    #         if cameraInfo.distortion_model == 'plumb_bob':
    #             self.intrinsics.model = rs2.distortion.brown_conrady
    #         elif cameraInfo.distortion_model == 'equidistant':
    #             self.intrinsics.model = rs2.distortion.kannala_brandt4
    #         self.intrinsics.coeffs = [i for i in cameraInfo.d]
    #     except CvBridgeError as e:
    #         print(e)
    #         return

def main(args=None):
    rclpy.init(args=args)
    node = Camera_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()
