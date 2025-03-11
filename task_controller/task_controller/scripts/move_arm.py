import time
from kinova_msgs.msg import ArmPose, GripperCommand
import rospy

def move_arm_to_position(x, y, z, roll, pitch, yaw):
    # Create an instance of the ArmPose message
    arm_pose = ArmPose()
    arm_pose.position.x = x
    arm_pose.position.y = y
    arm_pose.position.z = z
    arm_pose.orientation.roll = roll
    arm_pose.orientation.pitch = pitch
    arm_pose.orientation.yaw = yaw

    # Publish the arm pose to move the arm
    arm_pose_pub.publish(arm_pose)
    print(f"Moving arm to position: {x}, {y}, {z}, Roll: {roll}, Pitch: {pitch}, Yaw: {yaw}")

def grip_object():
    # Create a GripperCommand message to close the gripper
    gripper_command = GripperCommand()
    gripper_command.position = 0  # Close the gripper
    gripper_pub.publish(gripper_command)
    print("Gripping object...")

def release_object():
    # Create a GripperCommand message to open the gripper
    gripper_command = GripperCommand()
    gripper_command.position = 1  # Open the gripper
    gripper_pub.publish(gripper_command)
    print("Releasing object...")

def main():
    # Initialize ROS node
    rospy.init_node('kinova_arm_control')

    # Create publishers for arm pose and gripper commands
    global arm_pose_pub, gripper_pub
    arm_pose_pub = rospy.Publisher('/arm/pose', ArmPose, queue_size=10)
    gripper_pub = rospy.Publisher('/gripper/command', GripperCommand, queue_size=10)

    # Move arm to pick the object (position of the cube)
    move_arm_to_position(0.5, 0.0, 0.3, 0.0, 0.0, 0.0)  # Example coordinates (adjust as needed)
    time.sleep(2)  # Wait for arm to reach the position

    # Grip the object
    grip_object()
    time.sleep(2)  # Wait for gripper to close

    # Move arm to the goal position
    move_arm_to_position(0.8, 0.0, 0.3, 0.0, 0.0, 0.0)  # Goal coordinates
    time.sleep(2)

    # Release the object
    release_object()

    # Stop the gripper after releasing
    time.sleep(1)
    print("Task complete")

if __name__ == '__main__':
    main()

