import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class RobotNode(Node):

    def __init__(self):
        super().__init__('robot_node')

        # Define default action sequence
        default_action_sequence = ['move', 'perceive_plane', 'pick', 'place']

        # Execute default action sequence upon initialization
        for action in default_action_sequence:
            self.execute_action(action)
            time.sleep(5)  # Sleep for 5 seconds between actions

    def execute_action(self, action):
        action_topic = get_action_topic(action)
        self.get_logger().info('Executing action: %s' % action)
        # Publish message to start the action execution on the respective topic
        action_msg = String()
        action_msg.data = 'start'  # or any appropriate message to trigger action execution
        self.create_publisher(String, action_topic, 10).publish(action_msg)

def get_action_topic(action):
    if action == 'move':
        return 'start_move'
    elif action == 'perceive_plane':
        return 'start_perceive_plane'
    elif action == 'pick':
        return 'start_pick'
    elif action == 'place':
        return 'start_place'
    else:
        raise ValueError('Unknown action type: %s' % action)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
