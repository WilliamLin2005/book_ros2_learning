from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_learn',
            executable='tf2_main_imp',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('scan_input', '/scan_raw')]
        )
    ])