# System Architecture

## Overview

The current ROSMASTER X3 autonomy stack is organized around four subsystems:

1. Hardware bringup and sensor drivers
2. State estimation and SLAM
3. Frontier goal selection
4. Reactive wall/hallway following

The stack is intentionally modular. Mapping, exploration, and motion control are separate nodes so each piece can be tested independently.

## Data Flow

```text
Yahboom bringup
  -> /odom
  -> /imu/data
  -> driver_node

sllidar_ros2
  -> /scan

/odom + /imu/data
  -> robot_localization ekf_node
  -> odom -> base_link TF

/scan + odom/base_link TF
  -> slam_toolbox
  -> /map

/map + /odom
  -> frontier_explorer
  -> /exploration_goal

/scan + /exploration_goal
  -> hallway_follower
  -> /cmd_vel_safe
  -> driver_node
  -> robot motors
```

## Node Breakdown

### `bringup.launch.py`

Starts the hardware layer:

- `yahboomcar_bringup` for robot driver, odometry, IMU, and motor interface
- `sllidar_ros2` for LiDAR scans
- `tf2_ros static_transform_publisher` for `base_link` to `laser`
- `robot_localization ekf_node` with `config/ekf_params.yaml`

### `mapping.launch.py`

Starts mapping without hardware bringup:

- `scan_monitor` for lightweight `/scan`, `/odom`, and `/imu/data` diagnostics
- `slam_toolbox async_slam_toolbox_node` with `config/slam_params.yaml`

### `hallway_follower`

The main motion-control node. It subscribes to:

- `/scan`
- `/exploration_goal`

It publishes:

- `/cmd_vel_safe`

It handles wall following, corner handling, front obstacle stops, search mode, frontier-directed exploration, and recovery behavior. It is the only project node that should command `/cmd_vel_safe` during autonomy.

### `frontier_explorer`

The frontier selection node. It subscribes to:

- `/map`
- `/odom`

It publishes:

- `/exploration_goal`

It finds free cells adjacent to unknown map cells, clusters them, filters already visited targets, and publishes the best relative goal for the follower. It does not publish velocity commands.

### `hallway_detector`

Optional camera experiment node. It subscribes to:

- `/camera/color/image_raw`

It publishes:

- `/hallway_direction`
- `/hallway_steering_bias`

The current full autonomy launch does not depend on this node.

## Launch Composition

```text
bringup.launch.py
  hardware + LiDAR + static TF + EKF

mapping.launch.py
  scan_monitor + slam_toolbox

mapping_bringup.launch.py
  bringup.launch.py + mapping.launch.py

follower.launch.py
  bringup.launch.py + hallway_follower

slam_follower.launch.py
  bringup.launch.py + mapping.launch.py + hallway_follower

autonomy.launch.py
  bringup.launch.py + mapping.launch.py + hallway_follower + frontier_explorer
```

## Design Notes

- Keep `/cmd_vel_safe` ownership clear: `hallway_follower` commands motion.
- Keep exploration advisory: `frontier_explorer` publishes goals only.
- Use EKF output to stabilize the odometry transform consumed by SLAM Toolbox.
- Tune behavior through YAML files under `ros2_ws/src/rosmasterx3_slam/config/`.

## Future Work

- Add localization mode for reusing saved maps.
- Evaluate Nav2 integration once map reuse and localization are stable.
- Improve obstacle handling beyond reactive front-sector stopping.
- Revisit camera perception if vision is needed for semantic hallway cues.
