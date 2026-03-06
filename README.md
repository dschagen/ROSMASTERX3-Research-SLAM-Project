# ROSMASTERX3 Research SLAM Project

Research and development repo for building a self-driving ROSMASTER X3 robot capable of mapping one floor of a building with ROS 2 Foxy, LiDAR, SLAM Toolbox, and later Nav2 / exploration.

## Project Goal
Build a reliable workflow for:
- robot bringup
- safe velocity control
- LiDAR-based SLAM
- map saving
- navigation
- autonomous exploration

## Current Known-Good Stack
- Platform: ROSMASTER X3 / Yahboom stack
- ROS distro: ROS 2 Foxy
- LiDAR: `/scan` publishing at ~7 Hz
- Mapping: `slam_toolbox`
- Safe command path:
  - `/cmd_vel` -> `cmd_vel_watchdog` -> `/cmd_vel_safe` -> `driver_node`

## Repository Layout
```text
ROSMASTERX3-Research-SLAM-Project/
├── README.md
├── .gitignore
├── docs/
├── notes/
├── launch/
├── config/
├── scripts/
├── vendor/
├── maps/
├── bags/
└── ros2_ws/
    └── src/
```

## Immediate Next Steps
1. Move the existing Rosmaster helper library into `vendor/`.
2. Add your working workflow note to `docs/`.
3. Create your first custom ROS 2 package in `ros2_ws/src/`.
4. Add launch files for mapping and navigation.
5. Commit this cleaned structure as your project baseline.
