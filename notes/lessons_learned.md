# Lessons Learned

## ROS2 Integration

- QoS settings are critical for sensor data
- Camera topics often require sensor-data QoS
- Nodes may appear active without receiving data

---

## Debugging

- Always verify topic connections with:
  ros2 topic info -v

- Use:
  ros2 topic echo
  ros2 topic hz

to validate data flow

---

## System Design

- Separating perception and control simplifies debugging
- Modular nodes allow easier scaling
- Testing each component independently is essential

---

## Robotics Insights

- Reactive systems are simple but limited
- Stability requires smoothing and thresholds
- Real-world sensors introduce noise and variability

---

## Key Takeaway

A working system is not just code—it requires:
- correct architecture
- correct data flow
- careful parameter tuning
- iterative testing
