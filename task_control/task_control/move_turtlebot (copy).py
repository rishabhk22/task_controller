#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose, Point, Quaternion
from nav_msgs.msg import Odometry

from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import csv
import numpy as np
import time


class OdomCloseLoop(Node):
    def __init__(self, arm, gripper):
        super().__init__('kinova_turtle')
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
        self.pick_place = False
        self.reached_goal = False
        self.arm = arm
        self.gripper = gripper
        
    def odom_callback(self, msg):
        position_x = msg.pose.pose.position.x
        self.position_x_list.append(position_x)

        if len(self.position_x_list) > 2:
            distance_traveled = self.position_x_list[-1] - self.position_x_list[0]
            
            if self.moving_forward and distance_traveled >= self.target_distance:
                self.pub.publish(self.motion_stop)
                self.get_logger().info("Reached goal, stopping...")
                self.reached_goal = True
                self.kinova_setup()
                if(self.pick_place):
                    self.moving_forward = False
                    time.sleep(self.wait_time)  # Wait before reversing
                    self.reverse_started = True
                    self.position_x_list = [position_x]  # Reset tracking for reverse movement
                else:
                    self.get_logger().info("Waiting for pick place...")
            elif self.reverse_started and abs(distance_traveled) >= self.target_distance:
                self.pub.publish(self.motion_stop)
                self.get_logger().info("Returned to start, stopping...")
                rclpy.shutdown()
            
            else:
                if self.moving_forward and (self.reached_goal == False):
                    self.pub.publish(self.motion_move)
                elif self.reverse_started:
                    self.pub.publish(self.motion_reverse)
                
                if self.debug:
                    self.get_logger().info(f"Current position: {position_x}")

    def kinova_setup(self):
        
        if(self.reached_goal):
            self.gripper.move_to_position(0.0)
            self.gripper.close()

            # Load predefined poses
            csv_path = "/home/gixstudent/TECHIN516/kinova_ws/src/task_control/data.csv"
            poses = self.load_poses_from_csv(csv_path)

            # Execute movement sequence
            self.arm.go_vertical()
            self.move_arm_to_pose(self.arm, poses.get("point1"))
            self.move_arm_to_pose(self.arm, poses.get("point2"))
            self.gripper.move_to_position(0.7)
            self.move_arm_to_pose(self.arm, poses.get("point3"))
            ##self.arm.go_vertical()
            self.move_arm_to_pose(self.arm, poses.get("point4"))
            self.gripper.move_to_position(0.0)
            self.pick_place = True

    #kinova 
    def load_poses_from_csv(self, file_path):
        """
        Load pose data from a CSV file and return a dictionary of Pose objects.
        """
        poses = {}
        try:
            with open(file_path, "r") as csvfile:
                reader = csv.DictReader(csvfile)
                for row in reader:
                    pose = Pose(
                        position=Point(
                            x=float(row['pos_x']),
                            y=float(row['pos_y']),
                            z=float(row['pos_z'])
                        ),
                        orientation=Quaternion(
                            x=float(row['ori_x']),
                            y=float(row['ori_y']),
                            z=float(row['ori_z']),
                            w=float(row['ori_w'])
                        )
                    )
                    poses[row['name']] = pose
        except (FileNotFoundError, KeyError, ValueError) as e:
            print(f"Error loading CSV file: {e}")
        return poses

    def move_arm_to_pose(self, arm, pose, delay=1):
        """
        Move the robotic arm to a specified pose with an optional delay.
        """
        if pose:
            arm.inverse_kinematic_movement(pose)
            time.sleep(delay)
        else:
            print("Invalid pose provided.")

def main(args=None):
    """
    Main function to initialize ROS, control the robotic arm and gripper.
    """
    rclpy.init(args=args)
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()
    odom_cl = OdomCloseLoop(arm,gripper)



    #odom_cl.kinova_setup()
    # Ensure gripper is initially closed
    #arm.go_vertical()
    # gripper.move_to_position(0.0)
    # gripper.close()

    # # Load predefined poses
    # csv_path = "/home/gixstudent/TECHIN516/final/data.csv"
    # poses = odom_cl.load_poses_from_csv(csv_path)

    # # Execute movement sequence
    # odom_cl.move_arm_to_pose(arm, poses.get("point1"))
    # odom_cl.move_arm_to_pose(arm, poses.get("point2"))
    # gripper.move_to_position(0.7)
    # odom_cl.move_arm_to_pose(arm, poses.get("point3"))
    # arm.go_vertical()
    # odom_cl.move_arm_to_pose(arm, poses.get("point4"))
    # gripper.move_to_position(0.0)

    rclpy.spin(odom_cl)
    # Shutdown operations
    
    odom_cl.destroy_node()
    gripper.shutdown()
    arm.shutdown()
    rclpy.shutdown()



if __name__ == "__main__":
    main()





