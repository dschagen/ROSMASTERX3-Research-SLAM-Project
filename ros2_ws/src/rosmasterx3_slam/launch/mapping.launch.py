from launch import LaunchDescription
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('rosmasterx3_slam')

    return LaunchDescription([

        Node(
            package='rosmasterx3_slam',
            executable='scan_monitor',
            name='scan_monitor',
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'slam_params.yaml')]
        ),
    ])
