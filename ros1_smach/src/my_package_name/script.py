#!/usr/bin/env python3
#import roslib roslib.load_manifest('smach_tutorials')
import rospy
import smach
import smach_ros
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32, Int32

# define state Foo
class MonitorBatteryAndCollision(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['low_battery','collision', 'healthy'])
        # self.node = node
        self.battery_level_topic = rospy.Subscriber('battery_level_topic', Float32, self.battery_callback)
        print('hereeeeeeeeeeeeeee',self.battery_level_topic)
        self.collision_topic = rospy.Subscriber('collision_topic', LaserScan, self.collision_callback)
        self.battery_threshold = 30
        self.distance_threshold = 5
        self.battery_level = None
        self.collision_distance = None

    def battery_callback(self, msg):
        rospy.loginfo('Received battery level: %s', msg.data)
        self.battery_level = msg.data

    def collision_callback(self, msg):
        rospy.loginfo('Received collision distance: %s', min(msg.ranges))
        self.collision_distance = min(msg.ranges)

    def execute(self, userdata):
        rospy.loginfo('Executing state MonitorBatteryAndCollision')
        
        # Check battery level and collision status here
        if self.battery_level is not None and self.battery_level < self.battery_threshold:
            return 'low_battery'
        elif self.collision_distance is not None and self.collision_distance < self.distance_threshold:
            return 'collision'
        else:
            return 'healthy'


class Move(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['moving'])
        self.pub = rospy.Publisher('cmd_vel', Twist)

    def execute(self, userdata):
        rospy.loginfo('Executing state Move Base')
        twist = Twist()
        twist.linear.x = 0.5
        twist.angular.z = 0.0
        self.pub.publish(twist)
        return 'moving'


# define state Bar
class RotateBase(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['rotating'])
        self.pub = rospy.Publisher('cmd_vel', Twist)

    def execute(self, userdata):
        rospy.loginfo('Executing state Rotate Base')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.5
        self.pub.publish(twist)
        return 'rotating'


class StopMotion(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['stop'])
        self.pub = rospy.Publisher('cmd_vel', Twist)

    def execute(self, userdata):
        rospy.loginfo('Executing state Stop Base')
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.pub.publish(twist)
        return 'stop'


def main():
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['outcome'])

    # Open the container
    with sm:
        # Add states to the container
        smach.StateMachine.add('MonitorBatteryAndCollision', MonitorBatteryAndCollision(), 
                               transitions={'low_battery':'RotateBase', 'collision':'StopMotion', 'healthy':'Move'})
        smach.StateMachine.add('RotateBase', RotateBase(), 
                               transitions={'rotating':'MonitorBatteryAndCollision'})
        smach.StateMachine.add('StopMotion', StopMotion(), 
                               transitions={'stop':'MonitorBatteryAndCollision'})
        smach.StateMachine.add('Move', Move(), 
                               transitions={'moving':'MonitorBatteryAndCollision'})

    # Execute SMACH plan
    outcome = sm.execute()



if __name__ == '__main__':
    main()