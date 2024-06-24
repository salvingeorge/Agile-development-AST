#!usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class AvoidCollision(Node):
    def __init__(self):
        super().__init__('avoid_collision')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_distance = 1
    
    def laser_callback(self, msg):
        lidar_data = msg.ranges
        minimum_distance = min(lidar_data)
        self.get_logger().info("min distance: %f" % minimum_distance)
        if minimum_distance < self.safe_distance:
            self.stop()
        else:
            self.move()

    def stop(self):
        msg = Twist()
        msg.linear.x = 0.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('stopping robot')

    def move(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 0.0
        self.publisher.publish(msg)
        self.get_logger().info('moving')


def main(args=None):
    rclpy.init(args=args)
    avoid_collision = AvoidCollision()
    rclpy.spin(avoid_collision)
    avoid_collision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

