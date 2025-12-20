---
sidebar_label: "1.05 Lab — Hello Robot"
sidebar_position: 5
---

# Lab: Hello Robot

**Goal:** Create a ROS 2 package with a `hello_robot` node that heartbeats and publishes state; author a minimal URDF and visualize in `rviz2`.

## Steps

1. Create package

```bash
ros2 pkg create --build-type ament_python hello_robot_pkg
```

2. Add `hello_robot.py` (see module README) and update `setup.py` / `package.xml`.
3. Build with `colcon build` and source the install overlay.
4. Run the node: `ros2 run hello_robot_pkg hello_robot` or `python3 hello_robot.py`.
5. Verify topics: `ros2 topic list` and `ros2 topic echo /heartbeat` (or use the logger output).
6. Load URDF in a launch file using `robot_state_publisher` and view in `rviz2`.

## Checklist

- [ ] Node runs and logs heartbeat
- [ ] Topic `heartbeat` published at 1 Hz
- [ ] URDF loads in `rviz2` and visual looks reasonable
- [ ] Safety: motors are disabled during tests

:::danger
If you test on hardware, ensure E-stop and power sequencing are verified. Do not run actuators without supervision.
:::
