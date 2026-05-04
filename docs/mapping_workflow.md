# ROSMASTER X3 SLAM Mapping Workflow

## Overview

This document describes the current workflow for generating occupancy-grid maps with the ROSMASTER X3 ROS 2 Foxy stack.

The mapping path uses:

- RPLidar A1 scan data on `/scan`
- Yahboom odometry on `/odom`
- IMU data on `/imu/data`
- `robot_localization` EKF for filtered odometry
- `slam_toolbox` for mapping
- RViz for visualization

## Build and Source

```bash
cd ROSMASTERX3-Research-SLAM-Project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Start Mapping

The preferred mapping launch starts hardware, LiDAR, static TF, EKF, scan diagnostics, and SLAM Toolbox:

```bash
ros2 launch rosmasterx3_slam mapping_bringup.launch.py
```

Equivalent manual sequence:

```bash
ros2 launch rosmasterx3_slam bringup.launch.py
ros2 launch rosmasterx3_slam mapping.launch.py
```

## What Starts

```text
bringup.launch.py
  -> yahboomcar_bringup
  -> sllidar_ros2
  -> base_link to laser static TF
  -> robot_localization ekf_node

mapping.launch.py
  -> scan_monitor
  -> slam_toolbox async_slam_toolbox_node
```

## RViz

```bash
export DISPLAY=:0
ros2 run rviz2 rviz2
```

Recommended displays:

- Map: `/map`
- LaserScan: `/scan`
- TF
- RobotModel

Set the fixed frame to:

```text
map
```

## Mapping Strategy

For cleaner maps:

- Move slowly.
- Use short forward bursts.
- Stop fully between movements when the map begins to smear.
- Avoid fast turns.
- Prefer smooth surfaces over tile/grout when possible.
- Watch `/scan`, TF, and `/map` in RViz while tuning.

The current follower default cruise speed is `0.03 m/s`, configured in `config/follower_params.yaml`.

## Manual Motion Commands

Forward test command:

```bash
ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.00099}}"
```

Strong stop:

```bash
ros2 topic pub -r 20 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Key SLAM Parameters

Located in:

```text
ros2_ws/src/rosmasterx3_slam/config/slam_params.yaml
```

Current important settings:

```yaml
mode: mapping
resolution: 0.05
max_laser_range: 6.0
minimum_time_interval: 0.7
minimum_travel_distance: 0.15
minimum_travel_heading: 0.20
map_update_interval: 3.0
do_loop_closing: true
```

## Save a Map

While SLAM is running:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/map_name
```

If saving times out, move the robot slightly, stop it, and retry.

## Transfer a Map Into the Repo

```bash
cp /root/maps/map_name.* ~/Desktop/ROSMASTERX3-Research-SLAM-Project/maps/
```

Current saved maps in this repo include:

- `first_hallway_map`
- `hallway_run_01`
- `hallway_run_02`

## Useful Diagnostics

```bash
ros2 topic hz /scan
ros2 topic echo /map_metadata
ros2 topic echo /odom
ros2 topic echo /imu/data
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

## Current Status

- Hardware bringup, LiDAR, EKF, and SLAM are represented in launch files.
- Maps have been generated and saved under `maps/`.
- The current autonomy path can combine mapping, wall following, and frontier exploration through `autonomy.launch.py`.
