# Hallway Navigation System

## Goal

Enable the robot to autonomously navigate indoor hallways using only camera input.

---

## Approach

The hallway is divided into three regions:

- Left
- Center
- Right

The system identifies the most open direction based on edge density.

---

## Processing Pipeline

1. Capture image from `/camera/color/image_raw`
2. Convert to grayscale
3. Apply Gaussian blur
4. Apply Canny edge detection
5. Split image into three vertical regions
6. Count edge pixels in each region
7. Apply lower-half weighting
8. Apply center bias
9. Determine direction with minimum obstacle density

---

## Output

The system publishes:

/hallway_direction

Values:
- LEFT
- CENTER
- RIGHT
- NONE

---

## Control Mapping

| Direction | Motion |
|----------|--------|
| CENTER | Forward |
| LEFT | Turn left |
| RIGHT | Turn right |
| NONE | Stop |

---

## Stability Improvements

- Majority voting over recent frames
- Left/right similarity threshold
- Center bias to avoid oscillation
- Lower-half weighting for near-field accuracy

---

## Known Issues

- Sensitive to lighting changes
- Can misinterpret cluttered environments
- No obstacle avoidance beyond edges
