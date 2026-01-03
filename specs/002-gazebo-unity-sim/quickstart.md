# Quick Start: Gazebo & Unity Simulation Module

## Prerequisites

- ROS 2 Humble installed
- Gazebo Ignition Fortress installed
- Unity 2022 LTS (optional, for Unity chapters)

## Getting Started

### Gazebo Quick Start

```bash
# Launch sample world
ign gazebo -v 4 sample_world.sdf

# In another terminal, verify ROS 2 topics
ros2 topic list
```

### Unity Quick Start

1. Open Unity Hub
2. Create new project with Robotics template
3. Import ROS-TCP-Connector package
4. Configure endpoint to ROS 2

## Key Concepts

1. Physics engines simulate robot dynamics
2. Gazebo provides native ROS 2 integration
3. Unity offers high-fidelity rendering
4. Sensors can be simulated with noise models
