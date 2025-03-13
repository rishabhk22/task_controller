import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import csv


import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node
import time

#!/usr/bin/env python3



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
        
        self.target_distance = 1.7
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



#kinova 
def load_poses_from_csv(file_path):
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

def move_arm_to_pose(arm, pose, delay=1):
    """
    Move the robotic arm to a specified pose with an optional delay.
    """
    if pose:
        arm.inverse_kinematic_movement(pose)
        time.sleep(delay)
    else:
        print("Invalid pose provided.")

def main():
    """
    Main function to initialize ROS, control the robotic arm and gripper.
    """
    rclpy.init()
    #rclpy.init(args=args) #turtlebot
    odom_cl = OdomCloseLoop()
    rclpy.spin(odom_cl)


    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()

    # Ensure gripper is initially closed
    #arm.go_vertical()
    gripper.move_to_position(0.0)
    gripper.close()

    # Load predefined poses
    csv_path = "/home/gixstudent/TECHIN516/final/data.csv"
    poses = load_poses_from_csv(csv_path)

    # Execute movement sequence
    move_arm_to_pose(arm, poses.get("point1"))
    move_arm_to_pose(arm, poses.get("point2"))
    gripper.move_to_position(0.7)
    move_arm_to_pose(arm, poses.get("point3"))
    arm.go_vertical()
    move_arm_to_pose(arm, poses.get("point4"))
    gripper.move_to_position(0.0)

    # Shutdown operations
    gripper.shutdown()
    arm.shutdown()
    
    odom_cl.destroy_node()
    rclpy.shutdown()
    #rclpy.shutdown()


if __name__ == "__main__":
    main()





