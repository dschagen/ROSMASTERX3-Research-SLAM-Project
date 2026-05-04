# Lessons Learned

## ROS 2 Integration

- QoS settings are critical for sensor data.
- Camera topics often require sensor-data QoS.
- Nodes may appear active even when their subscriptions are not receiving data.
- Launch composition matters; keep hardware bringup separate from mapping and autonomy layers.

## Debugging

Always verify topic connections with:

```bash
ros2 topic info -v /topic_name
```

Use these commands to validate live data flow:

```bash
ros2 topic echo /topic_name
ros2 topic hz /topic_name
```

For transforms, use:

```bash
ros2 run tf2_ros tf2_echo odom base_link
ros2 run tf2_ros tf2_echo base_link laser
```

## System Design

- Separating mapping, exploration, and motion control simplifies debugging.
- Only one node should own `/cmd_vel_safe` during autonomy.
- Frontier exploration should publish goals, not direct velocity commands.
- YAML parameter files make field tuning faster and safer.

## Robotics Insights

- Reactive systems are simple and useful, but limited in cluttered spaces.
- Stability requires smoothing, thresholds, and slow speeds.
- Floor surface, LiDAR mounting, and sensor noise all affect behavior.
- Good maps require careful motion, not just correct code.

## Key Takeaway

A working robot system requires correct architecture, correct data flow, careful parameter tuning, and repeated real-world testing.
