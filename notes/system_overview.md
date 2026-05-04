# System Overview

## Architecture

The current system is composed of four main subsystems:

1. Hardware bringup and sensor drivers
2. Mapping and state estimation
3. Frontier exploration
4. Reactive wall/hallway following

Data flow:

```text
/scan -> slam_toolbox -> /map
/odom + /imu/data -> ekf_filter_node -> odom/base_link TF
/map + /odom -> frontier_explorer -> /exploration_goal
/scan + /exploration_goal -> hallway_follower -> /cmd_vel_safe -> driver_node
```

The optional camera hallway detector remains available for experiments, but the current autonomy launch path is LiDAR-first.

## Core Topics

| Topic | Description |
|------|-------------|
| `/scan` | LiDAR data |
| `/odom` | Robot odometry |
| `/imu/data` | IMU data |
| `/map` | SLAM occupancy grid |
| `/exploration_goal` | Frontier goal selected from the map |
| `/cmd_vel_safe` | Final velocity command sent to the robot driver |
| `/camera/color/image_raw` | Optional camera feed for vision experiments |

## Key Nodes

- `slam_toolbox`: mapping
- `ekf_filter_node`: odometry and IMU fusion
- `frontier_explorer`: frontier goal selection
- `hallway_follower`: wall following, obstacle response, and motion control
- `scan_monitor`: lightweight sensor-topic diagnostics
- `hallway_detector`: optional camera hallway perception
- `driver_node`: Yahboom robot actuation

## Design Philosophy

- Keep the stack modular and testable.
- Let only one project node own `/cmd_vel_safe` during autonomy.
- Use reactive control before adding heavier global planning.
- Tune behavior through YAML files instead of hard-coded constants.

## Current State

- Mapping launch files are in place and use SLAM Toolbox.
- EKF configuration is present for `/odom` plus `/imu/data`.
- Wall following and recovery behavior are parameterized.
- Frontier exploration is integrated through `autonomy.launch.py`.
