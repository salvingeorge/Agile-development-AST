#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MoveNode(Node):
    def __init__(self):
        super().__init__('move_node')
        self.publisher_ = self.create_publisher(String, 'move_status', 10)
        self.subscription = self.create_subscription(
            String,
            'move_action',
            self.move_callback,
            10)
        self.subscription  # prevent unused variable warning

    def move_callback(self, msg):
        self.get_logger().info(f'Received move action: {msg.data}')
        self.publish_status('executing')
        # Simulate move execution for 5 seconds
        self.get_logger().info('Simulating move execution...')
        self.publish_status('completed')

    def publish_status(self, status):
        msg = String()
        msg.data = status
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    move_node = MoveNode()
    rclpy.spin(move_node)
    move_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






#Human learning process
# import rclpy
# from std_msgs.msg import String 
# from rclpy.node import Node


# class Move(Node):
#     def __init__(self): 
#         super().__init__('minmal_publisher')
#         self.publisher_ = self.create_publisher(String, 'topic', 10)
#         self.timer = self.create_timer(0.5, self.timer_callback)

#     def timer_callback(self):
#         msg = String()
#         msg.data = 'HI PAOLA'
#         self.publisher_.publish(msg)
#         self.get_logger().info('Publishing....' %msg.data)
        
    


# def main(args=None):
#     rclpy.init(args=args)
#     move = Move()
#     rclpy.spin(move)

#     move.destroy_node()
#     rclpy.shutdown()