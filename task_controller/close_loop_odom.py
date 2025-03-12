#!/usr/bin/env python3

import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import time

class OdomCloseLoop(Node):
    def __init__(self):
        super().__init__('close_loop_odom')
        self.debug = True
        
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.position_x_list = []
        
        self.motion_move = Twist()
        self.motion_stop = Twist()
        self.motion_reverse = Twist()
        
        self.motion_move.linear.x = 0.15  # Forward velocity in m/s
        self.motion_stop.linear.x = 0.0
        self.motion_reverse.linear.x = -0.15  # Reverse velocity in m/s
        
        self.target_distance = 3.19
        self.moving_forward = True
        self.wait_time = 3  # Time to wait before reversing
        self.reverse_started = False
        
    def odom_callback(self, msg):
        position_x = msg.pose.pose.position.x
        self.position_x_list.append(position_x)

        if len(self.position_x_list) > 2:
            distance_traveled = self.position_x_list[-1] - self.position_x_list[0]
            
            if self.moving_forward and distance_traveled >= self.target_distance:
                self.pub.publish(self.motion_stop)
                self.get_logger().info("Reached goal, stopping...")
                self.moving_forward = False
                time.sleep(self.wait_time)  # Wait before reversing
                self.reverse_started = True
                self.position_x_list = [position_x]  # Reset tracking for reverse movement
            
            elif self.reverse_started and abs(distance_traveled) >= self.target_distance:
                self.pub.publish(self.motion_stop)
                self.get_logger().info("Returned to start, stopping...")
                rclpy.shutdown()
            
            else:
                if self.moving_forward:
                    self.pub.publish(self.motion_move)
                elif self.reverse_started:
                    self.pub.publish(self.motion_reverse)
                
                if self.debug:
                    self.get_logger().info(f"Current position: {position_x}")


def main(args=None):
    rclpy.init(args=args)
    odom_cl = OdomCloseLoop()
    rclpy.spin(odom_cl)
    odom_cl.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

