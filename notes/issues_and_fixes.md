# Issues and Fixes

## Map Saving Timeout

Cause:

- SLAM not publishing /map when stationary

Fix:

- Slightly move robot before saving map

## Robot Not Stopping

Cause:

- driver_node latches last velocity

Fix:

- publish zero velocity at high rate OR restart bringup

Command:

```bash
ros2 topic pub -r 50 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

## Launch File Confusion

Cause:

- The repo has project-level folders and an installable ROS package folder.

Fix:

- Keep runtime ROS 2 launch and config files under `ros2_ws/src/rosmasterx3_slam/`.
- Use the project-level folders for notes and documentation only.
