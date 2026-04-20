from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory('rosmasterx3_slam')

    return LaunchDescription([

        # 1) Hardware layer (bringup + LiDAR + TF)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'bringup.launch.py')
            )
        ),

        # 2) SLAM (scan_monitor + slam_toolbox)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg, 'launch', 'mapping.launch.py')
            )
        ),

        # 3) Wall follower — ONLY node writing to /cmd_vel_safe
        #    Handles: wall tracking, obstacle avoidance, exploration mode, recovery
        Node(
            package='rosmasterx3_slam',
            executable='hallway_follower',
            name='hallway_follower',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'follower_params.yaml')]
        ),

        # 4) Frontier explorer — reads /map, publishes /exploration_goal
        #    Never writes to /cmd_vel_safe
        Node(
            package='rosmasterx3_slam',
            executable='frontier_explorer',
            name='frontier_explorer',
            output='screen',
            parameters=[os.path.join(pkg, 'config', 'explorer_params.yaml')]
        ),
    ])
