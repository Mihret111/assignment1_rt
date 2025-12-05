from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # Start Turtlesim
        Node(
            package='turtlesim',
            executable='turtlesim_node',
            name='turtlesim'
        ),
        # Start Safety Node
        Node(
            package='assignment1_rt',
            executable='safety_node',
            name='safety_node',
            output='screen'
        ),
        # Start UI Node (in a new terminal usually, but here we run it in the same output or separate)
        # Since UI needs input, it's better to run it separately or use prefix to open in new term.
        # For simplicity in launch file, we often just launch background nodes.
        # But let's try to launch it. Note: stdin might be tricky.
        # Often better to leave UI to be run manually or use 'xterm -e'.
        
        # We will just launch the safety node and turtlesim here.
        # The spawner needs to run after turtlesim is ready.
    ])
