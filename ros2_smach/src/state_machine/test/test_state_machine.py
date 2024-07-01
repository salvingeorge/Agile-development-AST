import unittest
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from smach import StateMachine

from state_machine.state_machine import MonitorBatteryAndCollision, Move, RotateBase, StopMotion, main

'''refered from https://answers.ros.org/question/356180/ros2-creating-integration-tests-for-python-nodes/'''

class TestSmachStateMachine(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.test_node = rclpy.create_node('test_node')
        cls.test_node.get_logger().info('Node created')
    @classmethod
    def tearDownClass(cls):
        cls.test_node.get_logger().info('Destroying node')
        cls.test_node.destroy_node()
        rclpy.shutdown()
    
    def test_battery_callback(self):
        self.test_node.get_logger().info('Testing battery callback')
        state = MonitorBatteryAndCollision(self.test_node)
        msg = Float32()
        msg.data = 25.0
        state.battery_callback(msg)
        self.assertEqual(state.battery_level, 25.0)

    def test_laser_callback(self):
        self.test_node.get_logger().info('Testing laser callback')
        state = MonitorBatteryAndCollision(self.test_node)
        msg = LaserScan()
        msg.ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        state.collision_callback(msg)
        self.assertEqual(state.collision_distance, 1.0)

    def test_movement(self):
        self.test_node.get_logger().info('Testing movement')
        state = Move(self.test_node)
        self.assertEqual(state.execute(None), 'moving')

    def test_rotate_base(self):
        self.test_node.get_logger().info('Testing rotation')
        state = RotateBase(self.test_node)
        self.assertEqual(state.execute(None), 'rotating')

    def test_stop_motion(self):
        self.test_node.get_logger().info('Testing stopping motion')
        state = StopMotion(self.test_node)
        self.assertEqual(state.execute(None), 'stop')

    
if __name__ == '__main__':
    unittest.main()