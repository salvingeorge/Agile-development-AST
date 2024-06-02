import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class PerceivePlaneNode(Node):

    def __init__(self):
        super().__init__('perceive_plane_node')
        self.publisher_ = self.create_publisher(String, 'perceive_plane_topic', 10)
        self.subscription = self.create_subscription(
            String,
            'start_perceive_plane',
            self.start_perceive_plane_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.i = 0

    def start_perceive_plane_callback(self, msg):
        self.get_logger().info('Received: "%s"' % msg.data)
        self.execute_perceive_plane()

    def execute_perceive_plane(self):
        self.get_logger().info('Executing PERCEIVE_PLANE node %d' % self.i)
        # Simulate execution by waiting or performing some action
        # For demonstration purposes, let's sleep for 2 seconds
        self.get_logger().info('Executing...')
        time.sleep(2)  # Sleep for 2 seconds
        self.get_logger().info('Perceive Plane completed')
        self.i += 1
        # Publish completion message
        self.publish_completion()

    def publish_completion(self):
        msg = String()
        msg.data = 'Perceive Plane completed'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)


def main(args=None):
    rclpy.init(args=args)

    node = PerceivePlaneNode()

    rclpy.spin(node)

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
