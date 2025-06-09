# final/control_node.py

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from std_msgs.msg import String

class ControlNode(Node):
    def __init__(self):
        super().__init__('control_node')

        # Internal state
        self.patrol_active = False
        self.patrol_mode = 'basic'

        # Publisher to notify patrol system
        self.control_pub = self.create_publisher(String, 'patrol_control', 10)

        # Service: Start/Stop patrol
        self.create_service(SetBool, 'set_patrol_active', self.handle_patrol_active)

        # Service: Set patrol mode
        self.create_service(String, 'set_patrol_mode', self.handle_set_patrol_mode)

        self.get_logger().info('ControlNode started. Services: /set_patrol_active, /set_patrol_mode')

    def handle_patrol_active(self, request, response):
        self.patrol_active = request.data
        msg = String()
        msg.data = f"start" if self.patrol_active else "stop"
        self.control_pub.publish(msg)
        response.success = True
        response.message = f"Patrol {'started' if self.patrol_active else 'stopped'}"
        self.get_logger().info(response.message)
        return response

    def handle_set_patrol_mode(self, request, response):
        self.patrol_mode = request.data
        msg = String()
        msg.data = f"mode:{self.patrol_mode}"
        self.control_pub.publish(msg)
        response.data = True
        self.get_logger().info(f"Set patrol mode to: {self.patrol_mode}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()