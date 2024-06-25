import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



class Test(Node):
    def __init__(self):
        super().__init__('turning')
        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     '/scan',
        #     self.laser_callback,
        #     10
        # )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

    
    def laser_callback(self, msg):
        
        self.get_logger().info("turning: %f" % minimum_distance)