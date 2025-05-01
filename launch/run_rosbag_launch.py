from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='LinK3D', executable='link3d_rosbag', output='screen',
            parameters=[{'scan_line': 32}]
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', LaunchConfiguration('rviz_config')]
        )
    ])
