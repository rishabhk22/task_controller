import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Point, Quaternion
from .gen3lite_pymoveit2 import Gen3LiteArm, Gen3LiteGripper
import time
import csv



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
    arm = Gen3LiteArm()
    gripper = Gen3LiteGripper()

    # Ensure gripper is initially closed
    #arm.go_vertical()
    gripper.move_to_position(0.0)
    gripper.close()

    # Load predefined poses
    csv_path = "/home/gixstudent/TECHIN516/kinova_ws/src/task_control/data.csv"
    poses = load_poses_from_csv(csv_path)

    # Execute movement sequence
    move_arm_to_pose(arm, poses.get("point1"))
    move_arm_to_pose(arm, poses.get("point2"))
    gripper.move_to_position(0.7)
    move_arm_to_pose(arm, poses.get("point3"))
    #arm.go_vertical()
    move_arm_to_pose(arm, poses.get("point4"))
    gripper.move_to_position(0.0)

    # Shutdown operations
    gripper.shutdown()
    arm.shutdown()
    

    rclpy.shutdown()
    #rclpy.shutdown()


if __name__ == "__main__":
    main()





