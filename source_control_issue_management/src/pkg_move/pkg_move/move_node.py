import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class MyNode(Node):

    def __init__(self):
        super().__init__('move_node')
        self.publisher_ = self.create_publisher(String, 'chatter', 10)
        self.subscription = self.create_subscription(
            String,
            'start_move',
            self.start_move_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0

    def start_move_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        self.execute_move()

    def execute_move(self):
        self.get_logger().info('Executing MOVE node %d' % self.i)
        # Simulate execution by waiting or performing some action
        # For demonstration purposes, let's sleep for 3 seconds
        self.get_logger().info('Executing...')
        time.sleep(3)  # Sleep for 3 seconds
        self.get_logger().info('Move completed')
        self.i += 1
        # Publish completion message
        self.publish_completion()

    def publish_completion(self):
        msg = String()
        msg.data = 'Move completed'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = MyNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
