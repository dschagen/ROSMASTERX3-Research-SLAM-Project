from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
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
            parameters=[
                '/home/jetson/Desktop/ROSMASTERX3-Research-SLAM-Project/ros2_ws/src/rosmasterx3_slam/config/slam_params.yaml'
            ]
        )
    ])
