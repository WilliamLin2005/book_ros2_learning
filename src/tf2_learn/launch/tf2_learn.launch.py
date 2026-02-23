from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_learn',
            executable='tf2_main',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('input_scan', '/scan_raw')]
        )
    ])