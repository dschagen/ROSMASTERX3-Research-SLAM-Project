# Configuration

Runtime ROS 2 parameter files are stored inside the active package:

```text
ros2_ws/src/rosmasterx3_slam/config/
```

Current files:

- `slam_params.yaml`: SLAM Toolbox mapping parameters.
- `ekf_params.yaml`: `robot_localization` EKF parameters for `/odom` and `/imu/data`.
- `follower_params.yaml`: wall following, obstacle handling, exploration, and recovery behavior.
- `explorer_params.yaml`: frontier detection and goal-selection behavior.

This top-level `config/` folder is kept as a project note location. Add runtime YAML files to the ROS package config directory so `colcon` installs them with the package.
