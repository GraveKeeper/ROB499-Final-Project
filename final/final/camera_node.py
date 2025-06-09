import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')

        self.publisher = self.create_publisher(Image, '/camera/image_raw', 10)
        self.bridge = CvBridge()

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Simulated camera node publishing to /camera/image_raw')

    def timer_callback(self):
        # A test Dumb image
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.circle(img, (320, 240), 40, (0, 0, 255), -1)

        msg = self.bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'

        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()