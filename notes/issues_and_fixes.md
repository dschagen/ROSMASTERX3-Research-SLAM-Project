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
