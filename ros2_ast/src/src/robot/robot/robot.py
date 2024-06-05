import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')
        self.move_publisher_ = self.create_publisher(String, 'move_action', 10)
        self.perceive_plane_publisher_ = self.create_publisher(String, 'perceive_plane_action', 10)
        self.pick_publisher_ = self.create_publisher(String, 'pick_action', 10)
        self.place_publisher_ = self.create_publisher(String, 'place_action', 10)

        self.move_subscription_ = self.create_subscription(String, 'move_status', self.move_callback, 10)
        self.perceive_plane_subscription_ = self.create_subscription(String, 'perceive_plane_status', self.perceive_plane_callback, 10)
        self.pick_subscription_ = self.create_subscription(String, 'pick_status', self.pick_callback, 10)
        self.place_subscription_ = self.create_subscription(String, 'place_status', self.place_callback, 10)

        self.task_sequence = ['move', 'perceive_plane', 'pick', 'place']
        self.current_action_index = 0

        self.timer = self.create_timer(1.0, self.execute_next_action)

    def execute_next_action(self):
        if self.current_action_index < len(self.task_sequence):
            action = self.task_sequence[self.current_action_index]
            self.get_logger().info(f'Executing action: {action}')

            if action == 'move':
                self.move_publisher_.publish(String(data='move'))
            elif action == 'perceive_plane':
                self.perceive_plane_publisher_.publish(String(data='perceive_plane'))
            elif action == 'pick':
                self.pick_publisher_.publish(String(data='pick'))
            elif action == 'place':
                self.place_publisher_.publish(String(data='place'))

    def move_callback(self, msg):
        if msg.data == 'completed':
            self.current_action_index += 1

    def perceive_plane_callback(self, msg):
        if msg.data == 'completed':
            self.current_action_index += 1

    def pick_callback(self, msg):
        if msg.data == 'completed':
            self.current_action_index += 1

    def place_callback(self, msg):
        if msg.data == 'completed':
            self.current_action_index += 1
            self.get_logger().info('Task completed!')

def main(args=None):
    rclpy.init(args=args)
    robot_node = RobotNode()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
