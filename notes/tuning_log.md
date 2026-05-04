# Tuning Log

## Purpose

Track parameter adjustments and system behavior during testing.

## Early Camera-Follower Sessions

Initial hallway tests used the camera detector and discrete direction output.

Session 1:

- Forward speed: `0.02`
- Turn speed: `0.08`

Observations:

- Robot responded to direction changes.
- Slight oscillation during correction.
- Turns were somewhat aggressive.

Session 2:

- Reduced angular correction to `0.05`

Observations:

- Smoother motion.
- Slight delay in correction.

## Current LiDAR-Follower Baseline

The active autonomy stack now tunes `hallway_follower` through:

```text
ros2_ws/src/rosmasterx3_slam/config/follower_params.yaml
```

Current baseline values:

- `cruise_speed: 0.03`
- `search_speed: 0.02`
- `wall_target_m: 0.30`
- `kp_side: 0.20`
- `kd_side: 0.40`
- `angular_max: 0.20`
- `front_stop_m: 0.22`
- `front_slow_m: 0.45`
- `explore_speed: 0.08`
- `explore_turn_speed: 0.25`

## Next Tests

- Confirm wall-following stability in the target hallway.
- Tune `corner_open_m` and `corner_turn` for hallway openings.
- Validate frontier goal behavior during live mapping.
- Re-check `angle_offset_deg` if the LiDAR mounting changes.

## Notes

- Small changes to angular limits and PD gains have large effects.
- Stability depends heavily on clean LiDAR sectors and slow movement.
- Exploration should be tested with a reliable emergency stop command ready.
