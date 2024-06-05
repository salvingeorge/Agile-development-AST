#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PlaceNode(Node):
    def __init__(self):
        super().__init__('place_node')
        self.publisher_ = self.create_publisher(String, 'place_status', 10)
        self.subscription = self.create_subscription(
            String,
            'place_action',
            self.place_callback,
            10)
        self.subscription  # prevent unused variable warning

    def place_callback(self, msg):
        self.get_logger().info(f'Received place action: {msg.data}')
        self.publish_status('executing')
        # Simulate place execution for 4 seconds
        self.get_logger().info('Simulating place execution...')
        self.publish_status('completed')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    place_node = PlaceNode()
    rclpy.spin(place_node)
    place_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
