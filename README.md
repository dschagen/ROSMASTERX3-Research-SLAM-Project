# ROSMASTER X3 Research SLAM Project

This repository contains a full robotics pipeline for the Yahboom ROSMASTER X3 platform using ROS 2 Foxy. The system performs indoor mapping and autonomous navigation using LiDAR-based SLAM and camera-based hallway following.

---

## 🚀 Project Overview

This project builds a modular autonomy stack consisting of:

- LiDAR-based SLAM mapping  
- Camera-based perception using OpenCV  
- Reactive hallway navigation  
- ROS2 modular node architecture  

The system runs on a Jetson platform inside Docker using ROS 2 Foxy.

---

## 🧠 System Architecture
LiDAR → SLAM Toolbox → /map
Camera → Hallway Detector → /hallway_direction
Hallway Follower → /cmd_vel_safe → Driver Node → Robot Motors

---

## 📦 Current Features

### ✅ SLAM Mapping
- Uses `slam_toolbox`
- Publishes:
  - `/map`
  - `/map_metadata`
- Supports real-time indoor mapping

---

### ✅ Camera-Based Hallway Detection
- Input: `/camera/color/image_raw`
- Processing:
  - Canny edge detection  
  - Region-based scoring (left / center / right)  
  - Lower-half weighting for stability  
- Output:
  - `/hallway_direction` (`LEFT`, `CENTER`, `RIGHT`, `NONE`)

---

### ✅ Autonomous Hallway Following
- Node: `hallway_follower`
- Subscribes to `/hallway_direction`
- Publishes `/cmd_vel_safe`

Behavior:

| Direction | Action |
|----------|--------|
| CENTER   | Move forward |
| LEFT     | Steer left |
| RIGHT    | Steer right |
| NONE     | Stop |

---

### ✅ Modular ROS2 Design
- Separation of:
  - Perception (`hallway_detector`)
  - Control (`hallway_follower`)
  - Mapping (`slam_toolbox`)
- Designed for future Nav2 integration

---

## 🛠️ Setup

### Requirements
- ROS 2 Foxy  
- Docker (Jetson environment)  
- Yahboom ROSMASTER X3  
- Astra / Orbbec camera  
- LiDAR (A1)  

---

## ▶️ How to Run

### 1. Start Robot Bringup + SLAM
`ros2 launch rosmasterx3_slam mapping_bringup.launch.py`
### 2. Start Camera
`ros2 launch astra_camera astro_pro_plus.launch.xml`
### 3. Run Hallway Detector
`export QT_X11_NO_MITSHM=1`
`export DISPLAY=:0`
`ros2 run rosmasterx3_slam hallway_detector`
### 4. Run Hallway Follower
`ros2 run rosmasterx3_slam hallway_follower`
### 5. Emergencey Stop
`ros2 topic pub -r 50 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"`

## 🔍 Debugging

### Check topics:
`ros2 topic list`
### Check Camera:
`ros2 topic hz /camera/color/image_raw`
### Check hallway output:
`ros2 topic echo /hallway_direction`
### Check velocity:
`ros2 topic echo /cmd_vel_safe

## ⚙️ Tuning

### Inside hallway_follower.py:
`linear.x = 0.02`
`angular.z = 0.08`
### Adjust:
`- Lower angular.z → smoother motion`
`- Higher angular.z → faster correction`
`- Increase smoothing window → less jitter`

## 🧪 Current Limitations
`- No obstacle avoidance`
`- No global path planning`
`- Lighting sensitivity`
`- No dead-end detection`

## 📁 Repository Structure

ros2_ws/src/rosmasterx3_slam/
├── config/
├── launch/
├── rosmasterx3_slam/
│   └── nodes/
│       ├── hallway_detector.py
│       ├── hallway_follower.py
│       └── scan_monitor.py
├── package.xml
├── setup.py

## 👨‍💻 Authors
Daniel Schagen
Mark Halim
University of South Florida
Computer Science & Engineering
