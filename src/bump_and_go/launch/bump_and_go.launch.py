from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([ 
        Node(
            package='bump_and_go',
            executable='bump_and_go',
            output='screen',
            remappings=[
                ('scan_input', '/scan_raw'),
                ('twist_out', '/cmd_vel')
            ],
            parameters=[{'use_sim_time': True}]

        )
    ])
    
