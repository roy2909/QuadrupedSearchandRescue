#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as msg_Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np
import copy
from rclpy.qos import QoSProfile, QoSDurabilityPolicy
from sensor_msgs.msg import Image as msg_Image
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from std_srvs.srv import Empty
import tf2_geometry_msgs
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import pyrealsense2 as rs2
if (not hasattr(rs2, 'intrinsics')):
    import pyrealsense2.pyrealsense2 as rs2

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()


class Camera_subscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')
        self.bridge = CvBridge()
        self.depth_sub = self.create_subscription(
            msg_Image, '/camera/aligned_depth_to_color/image_raw', self.depth_callback, 1)
        self.depth_info_sub = self.create_subscription(
            CameraInfo, '/camera/color/camera_info', self.depth_info_callback, 1)
        self.sub = self.create_subscription(
            msg_Image, '/camera/color/image_raw', self.detect_human, 1)
        self.marker_publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.intrinsics = None
        self.pix = None
        self.pix_grade = None
        self.model = YOLO(
            '/home/rahulroy/winter_project_ws/src/unitree/image_yolo/image_yolo/yolov8n.pt')
        self.person_centroids = []
        self.yolov8_inference = Yolov8Inference()

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(msg_Image, "/inference_result", 1)
        # stroes coordinates of people detected
        self.people = []
        self.person_points = PointStamped()
        self._latest_color_img = None
        self._latest_depth_img = None
        self._latest_color_img_ts = None
        self.interfernce_ts = None

    def detect_human(self, data):
        image = bridge.imgmsg_to_cv2(data, "bgr8")
        # only detect person class
        classes = [0]
        conf_threshold = 0.5
        results = self.model.predict(
            source=image, classes=classes, conf=conf_threshold)

        # Assuming you only want information about the first detected person
        if results and results[0].boxes:
            box = results[0].boxes[0].xyxy[0].to('cpu').detach().numpy().copy()
            centroid = ((box[0] + box[2]) // 2, (box[1] + box[3]) // 2)
            self.person_centroids.append(centroid)
            self.inference_ts = copy.deepcopy(self._latest_color_img_ts)
            self.get_logger().info(f"Detected person at {self.inference_ts}")
            class_name = self.model.names[int(results[0].boxes[0].cls)]

            if class_name == 'person':
                # Calculate and log the distance
                x, y, z = self.depth_world(centroid[1], centroid[0])
                self.get_logger().info(f"Detected {class_name} at {centroid}")
                if x is not None and y is not None and z is not None:
                    self.get_logger().info(
                        f"Real world coordinates: {x}, {y}, {z}")
                    self.people.append([x, y, z])
                    self.create_marker(x, y, z)

        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model.predict(source=img, classes=classes, conf=0.5)

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
                    self.yolov8_inference.yolov8_inference.append(
                        self.inference_result)

        if len(self.yolov8_inference.yolov8_inference) > 0:
            annotated_frame = results[0].plot()
            img_with_overlay = cv2.addWeighted(
                img, 0.7, annotated_frame, 0.3, 0)
            img_msg = bridge.cv2_to_imgmsg(img_with_overlay)
            img_msg = bridge.cv2_to_imgmsg(annotated_frame)

            self.img_pub.publish(img_msg)
            self.yolov8_pub.publish(self.yolov8_inference)
            self.yolov8_inference.yolov8_inference.clear()

        return self.person_centroids

    def camera_callback(self, data):

        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self._latest_color_img = cv_image
            self._latest_color_img_ts = data.header.stamp
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return

    def create_marker(self, x, y, z):
        """
        Create a marker for visualization.

        Args:
        ----
            x (float): X-coordinate.
            y (float): Y-coordinate.
            z (float): Z-coordinate.

        Returns
        -------
            Marker: Visualization marker.

        """
        marker = Marker()

        marker.header.frame_id = "camera_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = z
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        self.marker_publisher.publish(marker)

    def depth_info_callback(self, cameraInfo):
        """
        Obtain depth camera information.

        Args:
        ----
            cameraInfo (CameraInfo): Camera information message.

        Returns
        -------
            None

        """
        try:
            if self.intrinsics:
                return
            self.intrinsics = rs2.intrinsics()
            self.get_logger().info(f"intrinsics: {self.intrinsics}")
            self.intrinsics.width = cameraInfo.width
            self.intrinsics.height = cameraInfo.height
            self.intrinsics.ppx = cameraInfo.k[2]
            self.intrinsics.ppy = cameraInfo.k[5]
            self.intrinsics.fx = cameraInfo.k[0]
            self.intrinsics.fy = cameraInfo.k[4]
            if cameraInfo.distortion_model == "plumb_bob":
                self.intrinsics.model = rs2.distortion.brown_conrady
            elif cameraInfo.distortion_model == "equidistant":
                self.intrinsics.model = rs2.distortion.kannala_brandt4
            self.intrinsics.coeffs = [i for i in cameraInfo.d]
        except CvBridgeError as e:
            print(e)
            return

    def depth_callback(self, data):
        """
        Obtain latest depth image.

        Args:
        ----
            data (Image): Depth image message.

        Returns
        -------
            None

        """
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            self._latest_depth_img = cv_image
        except CvBridgeError as e:
            print(e)
            return
        except ValueError as e:
            print(e)
            return

    def depth_world(self, x, y):
        """
            Convert pixel coordinates to real-world coordinates using depth information.

            Args:
            ----
                x (int): X-coordinate.
                y (int): Y-coordinate.

            Returns
            -------
                Tuple[float, float, float]: Real-world coordinates (x, y, z).

        """
        if (
            self.intrinsics and
            self._latest_depth_img is not None and
            0 <= x < self.intrinsics.width and
            0 <= y < self.intrinsics.height

        ):
            self.get_logger().info("processing request")

            depth_x = int(x)
            depth_y = int(y)
            depth = self._latest_depth_img[depth_x, depth_y]

            result = rs2.rs2_deproject_pixel_to_point(
                self.intrinsics, [y, x], depth)
            print(self.intrinsics)
            x_new, y_new, z_new = result[0], result[1], result[2]

            return x_new, y_new, z_new
        else:
            # Return some default values or handle the case as needed
            self.get_logger().warning("Unable to calculate real-world coordinates.")
            return 0.0, 0.0, 0.0


def main(args=None):
    rclpy.init(args=args)
    node = Camera_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()
