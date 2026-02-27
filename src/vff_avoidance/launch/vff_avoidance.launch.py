from launch import LaunchDescription
from launch_ros.actions import Node 

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vff_avoidance',
            executable='avoidance_node',
            output='screen',
            parameters=[{'use_sim_time': True}],
            remappings=[('output_twist', '/cmd_vel'),
                        ('input_scan', '/scan_raw')]

        )
    ])