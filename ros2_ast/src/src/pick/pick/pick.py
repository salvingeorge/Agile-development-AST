#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PickNode(Node):
    def __init__(self):
        super().__init__('pick_node')
        self.publisher_ = self.create_publisher(String, 'pick_status', 10)
        self.subscription = self.create_subscription(
            String,
            'pick_action',
            self.pick_callback,
            10)
        self.subscription  # prevent unused variable warning

    def pick_callback(self, msg):
        self.get_logger().info(f'Received pick action: {msg.data}')
        self.publish_status('executing')
        # Simulate pick execution for 2 seconds
        self.get_logger().info('Simulating pick execution...')
        self.publish_status('completed')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    pick_node = PickNode()
    rclpy.spin(pick_node)
    pick_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
