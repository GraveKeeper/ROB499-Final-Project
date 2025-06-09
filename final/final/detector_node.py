import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')

        self.bridge = CvBridge()
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.pub = self.create_publisher(String, 'detection_event', 10)

        self.last_detection = False
        self.get_logger().info('Detector node started. Listening on /camera/image_raw')

    def image_callback(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"cv_bridge error: {e}")
            return

        detected = self.detect_red_object(frame)

        if detected and not self.last_detection:
            self.pub.publish(String(data='red_object_detected'))
            self.get_logger().info('Red object detected!')
            self.last_detection = True
        elif not detected:
            self.last_detection = False

    def detect_red_object(self, frame):
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define the color range
        lower_red1 = (0, 70, 50)
        upper_red1 = (10, 255, 255)
        lower_red2 = (170, 70, 50)
        upper_red2 = (180, 255, 255)

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = mask1 | mask2

        red_pixels = cv2.countNonZero(red_mask)
        return red_pixels > 500

def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
