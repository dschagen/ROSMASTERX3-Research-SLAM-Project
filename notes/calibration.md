# Motion Calibration

## Problem
Robot drifted right during forward motion.

## Testing
Tested angular corrections:

- 0.0005 → slight right drift after ~6 ft
- 0.0006 → improved
- 0.00099 → best straight-line performance

## Result
Final value:
angular.z = 0.00099

## Observations
- Floor surface impacts drift (tile vs wood)
- Small corrections significantly affect SLAM quality
