#!usr/bin/env python

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

from collision_avoidance.utilities import *

class AvoidCollision(Node):
    def __init__(self, safe_d=1):
        super().__init__('avoid_collision')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10
        )

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.safe_distance = safe_d
    
    def laser_callback(self, msg):
        lidar_data = msg.ranges
        minimum_distance = min(lidar_data)



        grouped_points = None
        all_d_m_c_endPts = []
        d_m_c_endPts = []
        line_lengths = []
        distance_collection = []
        grouped_points = len(lidar_data) > 0 and online_line_detection_new(lidar_data)
        if grouped_points:
            for points_on_line in grouped_points:
                line_segment = Line(points_on_line[0], points_on_line[-1])
                end_pts = np.array([points_on_line[0], points_on_line[-1]])
                print("end-points: ", end_pts)
                m, c = line_segment.equation()
                center_wrt_laser = np.array([-0.45, 0])       # location of center of robot with respect to laser scanner
                d = line_segment.point_dist(center_wrt_laser) # distance fom the center of the Robile
                length = np.linalg.norm(end_pts[1] - end_pts[0])
                line_lengths.append(length)
                distance_collection.append(d)
                all_d_m_c_endPts.append([d, m, c, end_pts])
            best_line = np.min(distance_collection)
            best_line_idx = distance_collection.index(best_line)
            d_m_c_endPts = all_d_m_c_endPts[best_line_idx]

            slope = d_m_c_endPts[1]




        self.get_logger().info("min distance: %f" % minimum_distance)
        if minimum_distance < self.safe_distance:
            self.stop()
        

        if slope <-0.01 and minimum_distance < 2:
            self.turn_left()
        elif slope > 0.01 and minimum_distance < 2:
            self.turn_right()
        else:
            self.move()

    def stop(self, linear_x=0.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        self.get_logger().info('stopping robot')

    def move(self, linear_x=1.0, angular_z=0.0):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        self.get_logger().info('moving')

    def move_left(self, linear_x=0.5, angular_z=-0.5):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        self.get_logger().info('moving left')
    def move_right(self, linear_x=0.5, angular_z=-0.5):
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.publisher.publish(msg)
        self.get_logger().info('moving right')


def main(args=None):
    rclpy.init(args=args)
    avoid_collision = AvoidCollision()
    rclpy.spin(avoid_collision)
    avoid_collision.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

