# ROSMASTER X3 Research SLAM Project

This project contains the current ROS 2 Foxy research stack for the Yahboom ROSMASTER X3 platform. It supports LiDAR-based SLAM mapping, filtered odometry, wall/hallway following, and frontier-driven exploration.

The actively maintained ROS package is:

```text
ros2_ws/src/rosmasterx3_slam/
```

## Current Stack

- Robot hardware bringup through `yahboomcar_bringup`
- RPLidar startup through `sllidar_ros2`
- Static `base_link` to `laser` transform
- `robot_localization` EKF for `/odom` plus `/imu/data`
- `slam_toolbox` asynchronous mapping from `/scan`
- LiDAR wall follower publishing the final `/cmd_vel_safe`
- Frontier explorer publishing candidate goals on `/exploration_goal`
- Optional camera hallway detector for vision experiments

## Architecture

```text
/scan
  -> scan_monitor
  -> slam_toolbox
  -> /map

/odom + /imu/data
  -> ekf_filter_node
  -> odom -> base_link TF

/map + /odom
  -> frontier_explorer
  -> /exploration_goal

/scan + /exploration_goal
  -> hallway_follower
  -> /cmd_vel_safe
  -> Yahboom driver_node
  -> robot motors
```

The `hallway_follower` is the only project node that writes to `/cmd_vel_safe` during autonomy. The frontier explorer only selects goals and never commands the robot directly.

## Repository Layout

```text
ROSMASTERX3-Research-SLAM-Project/
  config/                 Project-level notes for configuration files
  docs/                   Architecture and workflow documentation
  launch/                 Project-level launch notes
  maps/                   Saved occupancy grids and run images
  notes/                  Calibration logs, lessons learned, and summaries
  ros2_ws/src/
    rosmasterx3_slam/     Active ROS 2 package
      config/             Runtime YAML parameters
      launch/             ROS 2 launch files
      rosmasterx3_slam/
        nodes/            Python ROS 2 nodes
      package.xml
      setup.py
  scripts/                Standalone test/helper scripts
  vendor/                 Optional third-party helper code
```

## ROS Package Entry Points

Installed console scripts from `rosmasterx3_slam`:

- `scan_monitor`
- `hallway_detector`
- `hallway_follower`
- `frontier_explorer`

## Launch Files

Run these from a sourced ROS 2 Foxy environment after building the workspace.

```bash
cd ROSMASTERX3-Research-SLAM-Project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

Hardware, LiDAR, static TF, and EKF:

```bash
ros2 launch rosmasterx3_slam bringup.launch.py
```

SLAM only, assuming hardware is already running:

```bash
ros2 launch rosmasterx3_slam mapping.launch.py
```

Hardware plus SLAM:

```bash
ros2 launch rosmasterx3_slam mapping_bringup.launch.py
```

Hardware plus wall following:

```bash
ros2 launch rosmasterx3_slam follower.launch.py
```

Hardware, SLAM, and wall following:

```bash
ros2 launch rosmasterx3_slam slam_follower.launch.py
```

Full autonomy with hardware, SLAM, wall following, and frontier exploration:

```bash
ros2 launch rosmasterx3_slam autonomy.launch.py
```

## Emergency Stop

Publish zero velocity to the safe command topic:

```bash
ros2 topic pub -r 50 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Useful Checks

```bash
ros2 topic list
ros2 topic hz /scan
ros2 topic echo /cmd_vel_safe
ros2 topic echo /exploration_goal
ros2 topic echo /map_metadata
ros2 run tf2_ros tf2_echo odom base_link
```

## Parameter Files

Runtime YAML files live in:

```text
ros2_ws/src/rosmasterx3_slam/config/
```

- `slam_params.yaml`: `slam_toolbox` mapping settings
- `ekf_params.yaml`: odometry and IMU fusion settings
- `follower_params.yaml`: wall following, obstacle handling, exploration, and recovery behavior
- `explorer_params.yaml`: frontier detection, goal selection, and visited-frontier filtering

## Maps

Saved maps and mapping screenshots live in `maps/`. Current map artifacts include:

- `first_hallway_map`
- `hallway_run_01`
- `hallway_run_02`
- `slam_final.png`
- April 2026 hallway run images and screenshots

## Optional Vision Node

`hallway_detector` subscribes to `/camera/color/image_raw` and publishes:

- `/hallway_direction`
- `/hallway_steering_bias`

The current autonomy launch path is LiDAR-first and does not depend on the camera detector.

## Documentation

- `docs/system_architecture.md`: current node/topic architecture
- `docs/mapping_workflow.md`: mapping procedure and map saving workflow
- `notes/`: calibration, tuning, hallway navigation, issues, lessons learned, and project summary material

## Authors

Daniel Schagen  
Mark Halim  
University of South Florida, Computer Science and Engineering
