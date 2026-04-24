from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    yahboom_dir = get_package_share_directory('yahboomcar_bringup')
    sllidar_dir = get_package_share_directory('sllidar_ros2')

    return LaunchDescription([

        # 1) Robot bringup — motors, IMU, odometry, driver_node
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(yahboom_dir, 'launch', 'yahboomcar_bringup_X3_launch.py')
            )
        ),

        # 2) LiDAR
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(sllidar_dir, 'launch', 'sllidar_launch.py')
            )
        ),

        # 3) Static TF: base_link → laser
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='laser_static_tf_pub',
            arguments=['0.10', '0.0', '0.12', '3.14159', '0', '0', 'base_link', 'laser'],
            output='screen'
        ),
    ])
