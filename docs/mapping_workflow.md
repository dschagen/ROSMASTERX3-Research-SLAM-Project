# ROSMASTERX3 SLAM Mapping Workflow

## Overview

This document describes the workflow used to generate occupancy grid maps using a ROS2-based SLAM pipeline on the ROSMASTER X3 platform.

The system uses:
- LiDAR (RPLidar A1)
- Wheel odometry
- IMU data
- `slam_toolbox` for mapping
- RViz for visualization

---

## System Architecture

```
/scan (LiDAR)
        ↓
/odom + /imu (robot state)
        ↓
slam_toolbox
        ↓
/map + /tf
        ↓
RViz (visualization)
```

---

## Prerequisites

Ensure the following are running:

- ROS2 Foxy environment
- Yahboom bringup stack
- LiDAR node
- Static TF between `base_link` and `laser`

---

## Launch Sequence

### 1. Start robot bringup

```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

---

### 2. Start LiDAR

```bash
ros2 launch sllidar_ros2 sllidar_launch.py
```

---

### 3. Publish static transform (LiDAR → base)

```bash
ros2 run tf2_ros static_transform_publisher 0.10 0.0 0.12 0 0 0 base_link laser
```

---

### 4. Start SLAM (custom package)

```bash
ros2 launch rosmasterx3_slam mapping.launch.py
```

---

### 5. Launch RViz

```bash
export DISPLAY=:0
ros2 run rviz2 rviz2
```

---

## RViz Setup

Add the following displays:

- **Map** → Topic: `/map`
- **LaserScan** → Topic: `/scan`
- **TF**
- **RobotModel**

Set Fixed Frame:
```
map
```

---

## Robot Motion Control

### Forward motion (calibrated)

```bash
ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.00099}}"
```

### Stop robot (strong stop)

```bash
ros2 topic pub -r 20 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## Mapping Strategy

To achieve high-quality maps:

- Move at **0.02 m/s**
- Use **short forward bursts**
- Fully stop between movements
- Avoid rapid turns
- Prefer smooth surfaces (wood > tile/grout)

---

## SLAM Parameters (Key Settings)

Located in:

```
ros2_ws/src/rosmasterx3_slam/config/slam_params.yaml
```

Key parameters:

```yaml
minimum_time_interval: 0.7
minimum_travel_distance: 0.05
minimum_travel_heading: 0.05
resolution: 0.05
```

---

## Saving a Map

While SLAM is running:

```bash
ros2 run nav2_map_server map_saver_cli -f /root/maps/map_name
```

If a timeout occurs:
- Slightly move the robot
- Immediately retry saving

---

## Transferring Map to Repository

```bash
cp /root/maps/map_name.* ~/Desktop/ROSMASTERX3-Research-SLAM-Project/maps/
```

Then commit:

```bash
git add maps/map_name.pgm maps/map_name.yaml
git commit -m "Add SLAM map"
git push
```

---

## Observations

- Floor surface significantly affects mapping accuracy
- Small angular correction (`0.00099`) was required for straight motion
- LiDAR operates at ~7 Hz, limiting update frequency
- Motion discipline is critical for map quality

---

## Current Status

- SLAM pipeline operational
- Maps successfully generated and saved
- RViz visualization working
- Motion calibration complete

---

## Next Steps

- Improve EKF tuning
- Add localization mode (map reuse)
- Explore loop closure optimization
- Automate motion control
