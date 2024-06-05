#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PerceivePlaneNode(Node):
    def __init__(self):
        super().__init__('perceive_plane_node')
        self.publisher_ = self.create_publisher(String, 'perceive_plane_status', 10)
        self.subscription = self.create_subscription(
            String,
            'perceive_plane_action',
            self.perceive_plane_callback,
            10)
        self.subscription  # prevent unused variable warning

    def perceive_plane_callback(self, msg):
        self.get_logger().info(f'Received perceive plane action: {msg.data}')
        self.publish_status('executing')
        # Simulate perceive plane execution for 3 seconds
        self.get_logger().info('Simulating perceive plane execution...')
        self.publish_status('completed')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    perceive_plane_node = PerceivePlaneNode()
    rclpy.spin(perceive_plane_node)
    perceive_plane_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
