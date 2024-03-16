import cv2
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge
from deepface import DeepFace

class Detect(Node):
    def __init__(self):
        super().__init__("detect")
        self.bridge = CvBridge()
        self._latest_color_img = None
        self._latest_color_img_ts = None

        self.sub1 = self.create_subscription(
            msg_Image, "/camera/color/image_raw", self.get_latest_frame, 1
        )

        self.pub = self.create_publisher(msg_Image, "/detected_image", 1)

        self.store = self.create_service(Empty, "detect", self.detect_face)

    def detect_face(self, request, response):
        if self._latest_color_img is not None:
            self.stored_img = cv2.imread("image.jpg")
            self.get_logger().info("got image")
            cv2.imwrite("image2.jpg", self._latest_color_img)
            result = DeepFace.verify(img1_path = "image.jpg", img2_path = "image2.jpg", model_name= "DeepID",enforce_detection=False)
            if result['verified'] == True:
                self.get_logger().info("MATCHES")
                image_with_text = self.add_text(self._latest_color_img, "MATCH")
                # Publish the image if matches
                self.publish_image(image_with_text)
            else:
                self.get_logger().info("NO MATCH")
       
        return response

    def get_latest_frame(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self._latest_color_img = cv_image
            self._latest_color_img_ts = data.header.stamp
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def publish_image(self, image):
        try:
            ros_image = self.bridge.cv2_to_imgmsg(image, "bgr8")
            self.pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {str(e)}")

    def add_text(self, image, text):
        # Choose font and position for the text
        font = cv2.FONT_HERSHEY_SIMPLEX
        bottom_left_corner = (10, 50)
        font_scale = 2  
        font_color = (0, 0, 255)  
        line_type = 2

        # Add text to the image
        cv2.putText(image, text, bottom_left_corner, font, font_scale, font_color, line_type)
        return image

def main(args=None):
    rclpy.init(args=args)
    node = Detect()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()