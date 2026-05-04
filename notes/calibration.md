# Motion Calibration

## Original Problem

The robot drifted right during forward manual motion tests.

## Straight-Line Testing

Tested angular corrections:

- `0.0005`: slight right drift after roughly 6 ft
- `0.0006`: improved
- `0.00099`: best straight-line performance during that session

Manual test command:

```bash
ros2 topic pub -r 10 /cmd_vel_safe geometry_msgs/msg/Twist "{linear: {x: 0.02, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.00099}}"
```

## Current Autonomy Tuning

The active follower now uses LiDAR wall-distance feedback instead of a fixed angular correction. Current runtime parameters are in:

```text
ros2_ws/src/rosmasterx3_slam/config/follower_params.yaml
```

Important values:

- `cruise_speed: 0.03`
- `wall_target_m: 0.30`
- `kp_side: 0.20`
- `kd_side: 0.40`
- `angular_max: 0.20`
- `front_stop_m: 0.22`
- `front_slow_m: 0.45`

## Observations

- Floor surface impacts drift and map quality.
- Small angular changes can noticeably affect SLAM quality.
- Slow motion and full stops help reduce map smearing.
- Wall-following calibration should be tested in the same hallway geometry used for mapping.
