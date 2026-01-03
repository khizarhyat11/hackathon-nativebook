# Quick Start: NVIDIA Isaac AI Brain Module

## Prerequisites

- NVIDIA GPU (RTX 3070 or better recommended)
- NVIDIA Isaac Sim 2023+ installed
- Isaac ROS packages installed
- ROS 2 Humble

## Getting Started

### Launch Isaac Sim

```bash
# Start Isaac Sim
./isaac-sim.sh

# In another terminal, verify ROS 2 connection
ros2 topic list
```

### Run Visual SLAM

```bash
# Launch Visual SLAM
ros2 launch isaac_ros_visual_slam isaac_ros_visual_slam.launch.py

# View the generated map in RViz
rviz2
```

### Navigate with Nav2

```bash
# Launch Nav2
ros2 launch nav2_bringup navigation_launch.py

# Send navigation goal from RViz or command line
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose
```

## Key Concepts

1. Isaac Sim provides GPU-accelerated simulation
2. Visual SLAM builds 3D maps from camera input
3. Nav2 handles path planning and obstacle avoidance
4. Isaac ROS provides optimized perception nodes
