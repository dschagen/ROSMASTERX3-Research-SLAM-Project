# Launch Files

Runtime ROS 2 launch files live in the active package:

```text
ros2_ws/src/rosmasterx3_slam/launch/
```

Current launch files:

- `bringup.launch.py`: hardware bringup, LiDAR, static TF, and EKF.
- `mapping.launch.py`: scan monitor and SLAM Toolbox.
- `mapping_bringup.launch.py`: hardware plus SLAM.
- `follower.launch.py`: hardware plus wall follower.
- `slam_follower.launch.py`: hardware, SLAM, and wall follower.
- `autonomy.launch.py`: hardware, SLAM, wall follower, and frontier explorer.

Common commands:

```bash
ros2 launch rosmasterx3_slam mapping_bringup.launch.py
ros2 launch rosmasterx3_slam autonomy.launch.py
```

Keep launch files in the package directory so they are installed by `setup.py` and found by `ros2 launch`.
