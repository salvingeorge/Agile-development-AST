import unittest
import rclpy

from rclpy.node import Node
from std_msgs.msg import Float32
from sensor_msgs.msg import Laserscan
from geometry_msgs.msg import Twist

from smach import StateMachine

from ros2_smach.src.state_machine import MonitorBatteryAndCollision, Move, RotateBase, StopMotion, main

'''source: https://answers.ros.org/question/356180/ros2-creating-integration-tests-for-python-nodes/'''

class TestSmachStateMachine():
    
    def setUP(self):
        rclpy.init()
        self.test_node = rclpy.create_node('test_node')

    def tearDown(self):
        self.test_node.destroy_node()
        rclpy.shutdown()
    
    def test_battery_callback(self):
        state = MonitorBatteryAndCollision(self.test_node)
        msg = Float32()
        msg.data = 25.0
        state.battery_callback(msg)
        self.assertEqual(state.battery_level, 25.0)

    def test_laser_callback(self):
        state = MonitorBatteryAndCollision(self.test_node)
        msg = Laserscan()
        msg.ranges = [1.0, 2.0, 3.0, 4.0, 5.0]
        state.laser_callback(msg)
        self.assertEqual(state.laser_data, 1.0)

    