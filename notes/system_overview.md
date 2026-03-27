# System Overview

## Architecture

The system is composed of three main subsystems:

1. Mapping (LiDAR-based SLAM)
2. Perception (camera-based hallway detection)
3. Control (velocity command generation)

Data flow:

LiDAR → SLAM Toolbox → /map  
Camera → hallway_detector → /hallway_direction  
hallway_follower → /cmd_vel_safe → driver_node  

---

## Core Topics

| Topic | Description |
|------|------------|
| /scan | LiDAR data |
| /map | Occupancy grid map |
| /camera/color/image_raw | Camera feed |
| /hallway_direction | Navigation decision |
| /cmd_vel_safe | Final velocity command |

---

## Key Nodes

- slam_toolbox → mapping
- hallway_detector → perception
- hallway_follower → control
- driver_node → robot actuation

---

## Design Philosophy

- Modular ROS2 architecture
- Separation of perception and control
- Reactive navigation first, planning later
- Incremental development and testing

---

## Current State

- Mapping working
- Camera perception stable
- Autonomous hallway following functional
- System ready for tuning and expansion
