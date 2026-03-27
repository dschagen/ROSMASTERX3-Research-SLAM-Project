ROSMASTER X3 Research SLAM Project

This repository contains a full robotics pipeline for the Yahboom ROSMASTER X3 platform using ROS 2 Foxy. The system performs indoor mapping and autonomous navigation using LiDAR-based SLAM and camera-based hallway following.

Project Overview:
## 📊 Current Status

✔ SLAM mapping operational  
✔ Camera perception working (~26 Hz)  
✔ Hallway detection stable  
✔ Autonomous hallway following functional  
⚠ Motion tuning in progress  

The system runs on a Jetson platform inside Docker using ROS 2 Foxy.

System Architecture
[ LiDAR ] --------> [ SLAM Toolbox ] --------> /map

[ Camera ] -------> [ Hallway Detector ] ----> /hallway_direction
                                           ↓
                                   [ Hallway Follower ]
                                           ↓
                                      /cmd_vel_safe
                                           ↓
                                      [ Driver Node ]
                                           ↓
                                      Robot Motors

Current Features
SLAM Mapping
Uses slam_toolbox
Publishes:
/map
/map_metadata
Supports real-time indoor mapping

Camera-Based Hallway Detection
Input: /camera/color/image_raw
Processing:
Canny edge detection
Region-based scoring (left / center / right)
Lower-half weighting for stability
Output:
/hallway_direction (LEFT, CENTER, RIGHT, NONE)
Autonomous Hallway Following
Node: hallway_follower
Subscribes to /hallway_direction
Publishes /cmd_vel_safe

Behavior:

CENTER  → Move forward  
LEFT    → Steer left  
RIGHT   → Steer right  
NONE    → Stop  

Modular ROS2 Design
Separation of:
Perception (hallway_detector)
Control (hallway_follower)
Mapping (slam_toolbox)
Designed for future Nav2 integration
🛠️ Setup
Requirements
ROS 2 Foxy
Docker (Jetson environment)
Yahboom ROSMASTER X3
Astra / Orbbec camera
LiDAR (A1)

How to Run
Start Robot Bringup + SLAM
ros2 launch rosmasterx3_slam mapping_bringup.launch.py
Start Camera
ros2 launch astra_camera astro_pro_plus.launch.xml
Run Hallway Detector
export QT_X11_NO_MITSHM=1
export DISPLAY=:0
ros2 run rosmasterx3_slam hallway_detector
Run Hallway Follower
ros2 run rosmasterx3_slam hallway_follower
Emergency Stop (IMPORTANT)
ros2 topic pub -r 50 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

Debugging
Check topics:
ros2 topic list

Check camera:
ros2 topic hz /camera/color/image_raw

Check hallway output:
ros2 topic echo /hallway_direction

Check velocity:
ros2 topic echo /cmd_vel_safe

Tuning
Inside hallway_follower.py:

linear.x = 0.02
angular.z = 0.08

Adjust:

Lower angular.z → smoother motion
Higher angular.z → faster correction
Increase smoothing window → less jitter

Current Limitations
No obstacle avoidance
No global path planning
Lighting sensitivity
No dead-end detection

Next Steps
Tune steering (PID control)
Add dead-end detection
Implement 90° turns
Integrate SLAM + camera navigation
Add watchdog safety node
Transition to Nav2

Repository Structure
ros2_ws/src/rosmasterx3_slam/
├── config/
├── launch/
├── rosmasterx3_slam/
│ └── nodes/
│ ├── hallway_detector.py
│ ├── hallway_follower.py
│ └── scan_monitor.py
├── package.xml
├── setup.py

Key Concepts:
SLAM
Computer Vision (OpenCV)
Reactive Control
ROS2 Architecture

## 🧩 Future System Vision

This project will evolve into a full autonomous navigation system combining:

- Visual perception (camera)
- LiDAR-based mapping
- Localization and path planning (Nav2)

Goal: robust indoor navigation in unknown environments.

Authors:
Daniel Schagen
Mark Halim
University of South Florida
Computer Science & Engineering
