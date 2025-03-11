import os
import time

# Define velocity and time values
speed = 0.15  # meters per second
distance = 1.5  # meters
time_value = distance / speed  # Calculate time based on speed and distance

# Move forward for 1.5 meters using an open-loop strategy
timenow = time.time()

# Command to move the TurtleBot in a straight line
os.system(f"ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{{linear: {{x: {speed}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: 0.0}}}}'")

print(f"Speed: {speed}, Start timer: {timenow}")

# The loop will run for the calculated time (time_value)
while time.time() - timenow < time_value:
    pass

# Stop the robot after moving
os.system("ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}'")
print(f"Speed 0.0 Stop timer: {time.time() - timenow}")

