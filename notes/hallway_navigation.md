# Hallway Navigation System

## Goal

Enable the robot to move through indoor hallways at low speed while maintaining a usable wall distance, avoiding front obstacles, and supporting frontier-directed exploration during mapping.

## Current Approach

The current follower is LiDAR-based. `hallway_follower` subscribes to `/scan` and computes:

- front obstacle distance
- active side-wall distance
- diagonal/corner distance
- opposite-wall availability

It publishes final motion commands on `/cmd_vel_safe`.

## Controller Modes

- `follow`: maintain the active wall target distance.
- `corner`: arc through an opening when the side wall disappears.
- `turn`: rotate when the front sector is blocked.
- `settle`: pause after a turn before reacquiring the wall.
- `search`: move slowly while looking for a wall.
- `explore`: move toward a frontier goal from `/exploration_goal`.
- `recovery_backup` and `recovery_turn`: escape short stuck cases.

## Key Parameters

Runtime tuning lives in:

```text
ros2_ws/src/rosmasterx3_slam/config/follower_params.yaml
```

Important groups:

- wall selection: `wall_side`, `follow_any_wall`, `wall_target_m`
- speeds: `cruise_speed`, `search_speed`, `linear_scale`
- side-wall PD control: `kp_side`, `kd_side`, `angular_max`
- front obstacle handling: `front_stop_m`, `front_slow_m`
- corner behavior: `corner_open_m`, `corner_turn`, `corner_timeout_sec`
- exploration behavior: `explore_speed`, `explore_turn_speed`, `explore_goal_dist`
- recovery behavior: `recovery_backup_sec`, `recovery_turn_sec`

## Optional Camera Detector

`hallway_detector` is still available as a camera experiment. It subscribes to `/camera/color/image_raw` and publishes:

- `/hallway_direction`
- `/hallway_steering_bias`

That detector uses edge density over image regions, majority voting, center bias, and lower-half weighting. It is not part of the current full autonomy launch.

## Known Issues

- Reactive control can struggle in cluttered or open areas without a reliable wall.
- Floor surface and LiDAR mounting affect consistency.
- Front obstacle handling is local and does not replace full path planning.
- Frontier goals are advisory; the follower may return to wall following when a wall is reacquired.
