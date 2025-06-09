import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.qos import QoSProfile
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import yaml
import os

class PatrolNode(Node):
    def __init__(self):
        super().__init__('patrol_node')

        self.get_logger().info('Starting patrol_node...')

        self.nav_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Sub to control commands from control_node
        self.control_sub = self.create_subscription(
            String, 'patrol_control', self.control_callback, QoSProfile(depth=10)
        )

        self.waypoints = []
        self.current_index = 0
        self.patrolling = False
        self.mode = 'basic'

        # Load waypoints from param file if given Will try and add this later when this is working
        waypoint_file = self.declare_parameter('waypoint_file', '').value
        if waypoint_file and os.path.exists(waypoint_file):
            self.load_waypoints(waypoint_file)
            self.get_logger().info(f"Loaded {len(self.waypoints)} waypoints from: {waypoint_file}")
        else:
            self.get_logger().warn('No valid waypoint file provided. Use dynamic input or set a valid param.')

        self.timer = self.create_timer(1.0, self.timer_callback)

    def load_waypoints(self, path):
        with open(path, 'r') as f:
            data = yaml.safe_load(f)
            self.waypoints = data['waypoints']

    def control_callback(self, msg):
        data = msg.data
        if data == 'start':
            self.get_logger().info('Patrol started.')
            self.patrolling = True
        elif data == 'stop':
            self.get_logger().info('Patrol stopped.')
            self.patrolling = False
        elif data.startswith('mode:'):
            self.mode = data.split(':', 1)[1]
            self.get_logger().info(f'Patrol mode set to: {self.mode}')

    def timer_callback(self):
        if self.patrolling and self.waypoints:
            if not self.nav_action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().warn('Waiting for NavigateToPose action server...')
                return

            waypoint = self.waypoints[self.current_index]
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = PoseStamped()
            goal_msg.pose.header.frame_id = 'map'
            goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
            goal_msg.pose.pose.position.x = waypoint['x']
            goal_msg.pose.pose.position.y = waypoint['y']
            goal_msg.pose.pose.orientation.z = waypoint.get('z', 0.0)
            goal_msg.pose.pose.orientation.w = waypoint.get('w', 1.0)

            self.get_logger().info(f'Navigating to waypoint {self.current_index + 1}/{len(self.waypoints)}...')
            self._send_goal(goal_msg)

            self.current_index = (self.current_index + 1) % len(self.waypoints)
            self.patrolling = False  # Must Wait for success before restarting

    def _send_goal(self, goal_msg):
        send_goal_future = self.nav_action_client.send_goal_async(goal_msg)
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn('Goal was rejected')
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'Reached waypoint! Status: {result}')
        self.patrolling = True  # Continue to next waypoint


def main(args=None):
    rclpy.init(args=args)
    node = PatrolNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()