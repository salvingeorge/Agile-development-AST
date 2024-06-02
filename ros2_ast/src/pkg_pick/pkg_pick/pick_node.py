import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class PickNode(Node):

    def __init__(self):
        super().__init__('pick_node')
        self.publisher_ = self.create_publisher(String, 'pick_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'start_pick',
            self.start_pick_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0

    def start_pick_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        self.execute_pick()

    def execute_pick(self):
        self.get_logger().info('Executing PICK node %d' % self.i)
        # Simulate execution by waiting or performing some action
        # For demonstration purposes, let's sleep for 2 seconds
        self.get_logger().info('Executing...')
        time.sleep(2)  # Sleep for 2 seconds
        self.get_logger().info('Pick completed')
        self.i += 1
        # Publish completion message
        self.publish_completion()

    def publish_completion(self):
        msg = String()
        msg.data = 'Pick completed'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = PickNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
