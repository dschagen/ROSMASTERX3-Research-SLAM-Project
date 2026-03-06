# Yahboom X3 Mapping Workflow

## Startup order
1. Bringup
2. Watchdog
3. LiDAR
4. SLAM Toolbox
5. Motion test

## Bringup
```bash
ros2 launch yahboomcar_bringup yahboomcar_bringup_X3_launch.py
```

## Watchdog
```bash
ros2 run yahboom_safety cmd_vel_watchdog --ros-args -r /vel_raw:=/cmd_vel_safe
```

## LiDAR
```bash
ros2 launch yahboomcar_laser laser_Warning_a1_X3.launch.py
```

## SLAM Toolbox
```bash
ros2 launch slam_toolbox online_async_launch.py
```

## Motion test
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.12, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}" --times 10
```

## Save map
```bash
ros2 service call /slam_toolbox/save_map slam_toolbox/srv/SaveMap "{name: {data: '/root/yahboomcar_ros2_ws/yahboomcar_ws/maps/hallway_map_v1'}}"
```
