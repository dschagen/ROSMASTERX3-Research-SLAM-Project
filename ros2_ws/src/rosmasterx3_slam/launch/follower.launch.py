from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('rosmasterx3_slam')

    return LaunchDescription([

        # Hardware layer (bringup + LiDAR + TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'bringup.launch.py')
            )
        ),

        # Wall follower node
        Node(
            package='rosmasterx3_slam',
            executable='hallway_follower',
            name='hallway_follower',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'follower_params.yaml')]
        ),
    ])
