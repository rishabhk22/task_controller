import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        # First node: close_loop_odom from closed_loop_control
        launch_ros.actions.Node(
            package='closed_loop_control',  # Package name
            executable='close_loop_odom',   # Executable name
            name='close_loop_odom'
        ),

        # Second node: move_arm from task_control
        launch_ros.actions.Node(
            package='task_control',  # Package name
            executable='move_arm',   # Executable name
            name='move_arm'
        )
    ])

