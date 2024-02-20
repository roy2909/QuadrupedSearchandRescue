import cv2
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from sensor_msgs.msg import Image as msg_Image
from cv_bridge import CvBridge

class Identify(Node):
    def __init__(self):
        super().__init__("identify")
        self.bridge = CvBridge()
        self._latest_color_img = None
        self._latest_color_img_ts = None

        self.sub1 = self.create_subscription(
            msg_Image, "/camera/color/image_raw", self.get_latest_frame, 1
        )

        self.store = self.create_service(Empty, "store", self.store_face)

    def store_face(self, request, response):
        if self._latest_color_img is not None:
            cv2.imwrite("image.jpg", self._latest_color_img)
            self.get_logger().info("Image stored successfully")
        else:
            self.get_logger().info("No image available to store")

        return response

    def get_latest_frame(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            self._latest_color_img = cv_image
            self._latest_color_img_ts = data.header.stamp
        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = Identify()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
