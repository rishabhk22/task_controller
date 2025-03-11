import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package="task_controller",  # Package name
            executable="move_turtlebot",  # Python executable name
            output="screen",  # Output to screen
        ),
        launch_ros.actions.Node(
            package="task_controller",  # Package name
            executable="move_arm",  # Python executable name
            output="screen",  # Output to screen
        ),
    ])

