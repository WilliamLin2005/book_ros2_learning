import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python import get_package_share_directory

def generate_launch_description():
    param_file=os.path.join(
        get_package_share_directory('my_package'),
        'config',
        'parameter.yaml'
    )
    return LaunchDescription(
        [
            Node(
                package='my_package',
                executable='parameters',
                parameters=[param_file],
                output='screen'
            )
        ]
    )