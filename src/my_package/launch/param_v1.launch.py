from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription(
        [
        Node(
            package='my_package',
            executable='parameters',
            parameters=[{
                'num_particles': 300,
                'topic_name': ['scan', 'image'],
                'topic_type': ['sensor_msgs/msg/LaserScan', 'sensor_msgs/msg/Image']
            }],
            output='screen'
        )
        ]
    )