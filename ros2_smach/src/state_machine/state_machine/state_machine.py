#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from sensor_msgs.msg import LaserScan
import smach

# Define state MonitorBatteryAndCollision


class MonitorBatteryAndCollision(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['low_battery', 'collision', 'healthy', 'avoid_collision'])
        self.node = node
        self.battery_threshold = 30
        self.distance_threshold = 1
        self.avoid_distance = 2
        self.battery_level = None
        self.collision_distance = None
        self.battery_sub = self.node.create_subscription(
            Float32, 'battery_level_topic', self.battery_callback, 10)
        self.collision_sub = self.node.create_subscription(
            LaserScan, '/scan', self.collision_callback, 10)

    def battery_callback(self, msg):
        self.node.get_logger().info(f'Received battery level: {msg.data}')
        self.battery_level = msg.data

    def collision_callback(self, msg):
        min_distance = min(msg.ranges)
        self.node.get_logger().info(f'Received collision distance: {min_distance}')
        self.collision_distance = min_distance

    def execute(self, userdata):
        self.node.get_logger().info('Executing state MonitorBatteryAndCollision')
        # Wait a short time to ensure callbacks are processed
        rclpy.spin_once(self.node, timeout_sec=0.1)
        # Check battery level and collision status here
        if self.battery_level is not None and self.battery_level < self.battery_threshold:
            return 'low_battery'
        elif self.collision_distance is not None and self.collision_distance < self.distance_threshold:
            return 'collision'
        elif self.collision_distance is not None and self.collision_distance < self.avoid_distance:
            return 'avoid_collision'
        else:
            return 'healthy'


class Move(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['moving'])
        self.node = node
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)

    def execute(self, userdata):
        self.node.get_logger().info('Executing state Move')
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.pub.publish(twist)
        return 'moving'


# Define state RotateBase
class RotateBase(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['rotating'])
        self.node = node
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)

    def execute(self, userdata):
        self.node.get_logger().info('Executing state Rotate Base')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.pub.publish(twist)
        return 'rotating'


class StopMotion(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['stop'])
        self.node = node
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)

    def execute(self, userdata):
        self.node.get_logger().info('Executing state Stop Base')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        return 'stop'
    
class AvoidObstacle(smach.State):
    def __init__(self, node):
        smach.State.__init__(self, outcomes=['avoiding'])
        self.node = node
        self.pub = self.node.create_publisher(Twist, 'cmd_vel', 10)

    def execute(self, userdata):
        self.node.get_logger().info('Executing state Avoid Obstacle')
        twist = Twist()
        twist.linear.x = -0.2
        twist.angular.z = 0.5
        self.pub.publish(twist)
        return 'avoiding'


def main():
    rclpy.init()
    node = Node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MonitorBatteryAndCollision', MonitorBatteryAndCollision(node),
                               transitions={'low_battery': 'RotateBase', 'collision': 'StopMotion', 'avoid_collision': 'AvoidObstacle','healthy': 'Move'})
        smach.StateMachine.add('RotateBase', RotateBase(node),
                               transitions={'rotating': 'MonitorBatteryAndCollision'})
        smach.StateMachine.add('StopMotion', StopMotion(node),
                               transitions={'stop': 'MonitorBatteryAndCollision'})
        smach.StateMachine.add('Move', Move(node),
                               transitions={'moving': 'MonitorBatteryAndCollision'})
        smach.StateMachine.add('AvoidObstacle', AvoidObstacle(node),
                               transitions={'avoiding': 'MonitorBatteryAndCollision'})

    # Execute SMACH plan
    outcome = sm.execute()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
