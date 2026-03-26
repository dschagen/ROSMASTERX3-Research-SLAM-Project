from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    use_rviz = LaunchConfiguration('use_rviz')

    yahboom_bringup_dir = get_package_share_directory('yahboomcar_bringup')
    sllidar_dir = get_package_share_directory('sllidar_ros2')
    rosmasterx3_slam_dir = get_package_share_directory('rosmasterx3_slam')

    yahboom_bringup_launch = os.path.join(
        yahboom_bringup_dir,
        'launch',
        'yahboomcar_bringup_X3_launch.py'
    )

    sllidar_launch = os.path.join(
        sllidar_dir,
        'launch',
        'sllidar_launch.py'
    )

    mapping_launch = os.path.join(
        rosmasterx3_slam_dir,
        'launch',
        'mapping.launch.py'
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Whether to launch RViz'
        ),

        # 1) Robot bringup
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(yahboom_bringup_launch)
        ),

        # 2) LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(sllidar_launch)
        ),

        # 3) Static TF: base_link -> laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_tf_pub',
            arguments=['0.10', '0.0', '0.12', '0', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),

        # 4) Your SLAM mapping stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mapping_launch)
        ),
    ])
