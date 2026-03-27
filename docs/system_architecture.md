# System Architecture

## Overview

The ROSMASTER X3 system is composed of three primary subsystems:

1. Mapping (LiDAR-based SLAM)
2. Perception (camera-based hallway detection)
3. Control (velocity command generation)

---

## Data Flow
LiDAR → /scan → slam_toolbox → /map

Camera → /camera/color/image_raw → hallway_detector → /hallway_direction
↓
hallway_follower
↓
/cmd_vel_safe
↓
driver_node
↓
Robot Motors

---

## Node Breakdown

### Mapping
- Node: `slam_toolbox`
- Input: `/scan`
- Output: `/map`, `/map_metadata`

---

### Perception
- Node: `hallway_detector`
- Input: `/camera/color/image_raw`
- Output: `/hallway_direction`

---

### Control
- Node: `hallway_follower`
- Input: `/hallway_direction`
- Output: `/cmd_vel_safe`

---

### Actuation
- Node: `driver_node`
- Input: `/cmd_vel_safe`
- Output: motor commands
---

## Key Design Decisions

- Modular architecture (separate perception and control)
- Reactive navigation before global planning
- Use of sensor-data QoS for camera streams
- Emphasis on real-time performance

---

## Future Integration

Planned upgrades:

- Localization (AMCL / particle filter)
- Path planning (Nav2)
- Sensor fusion (camera + LiDAR)
- Obstacle avoidance

---

## Summary

The system is designed to incrementally evolve from reactive navigation
