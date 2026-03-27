# Tuning Log

## Purpose

Track parameter adjustments and system behavior during testing.

---

## Session 1

- Forward speed: 0.02
- Turn speed: 0.08

Observations:
- Robot responds correctly to direction
- Slight oscillation during correction
- Turns slightly aggressive

---

## Session 2

- Reduced angular.z to 0.05

Observations:
- Smoother motion
- Slight delay in correction

---

## Session 3 (Planned)

- Test smoothing window size increase
- Adjust diff_threshold in detector
- Evaluate behavior in narrow hallway

---

## Notes

- Small changes in angular.z have large effects
- Stability depends heavily on detector smoothing
- Lower camera position improves edge detection
