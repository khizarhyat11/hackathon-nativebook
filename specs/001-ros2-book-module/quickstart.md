# Quick Start: ROS 2 Book Module

## Prerequisites

- ROS 2 Humble or later installed
- Python 3.8+ environment
- Basic understanding of Python programming
- Docusaurus installed for content viewing

## Getting Started

### 1. Navigate to the Module Documentation

```bash
cd my-website/docs/module-01-ros2/
```

### 2. View the Content Structure

The ROS 2 module consists of 4 chapters:

1. **architecture.mdx** - ROS 2 Architecture Fundamentals
2. **communication.mdx** - Nodes, Topics, and Services
3. **ai-integration.mdx** - AI Agent Integration
4. **urdf-modeling.mdx** - URDF for Humanoid Robots

### 3. Run Example Code

All code examples are located in the `examples/ros2/` directory:

```bash
# Publisher/Subscriber example
cd examples/ros2/publisher_subscriber/
python3 minimal_publisher.py

# In another terminal
python3 minimal_subscriber.py
```

### 4. Validate URDF Models

```bash
# Check URDF syntax
check_urdf humanoid_robot.urdf

# Visualize in RViz
ros2 launch urdf_tutorial display.launch.py model:=humanoid_robot.urdf
```

## Key Learning Objectives

After completing this module, you will be able to:

1. Explain ROS 2 architecture including nodes, executors, and DDS
2. Implement publisher/subscriber and service/client communication patterns
3. Connect AI agents to ROS 2 controllers using rclpy
4. Create and validate URDF models for humanoid robots

## Verification

To verify your understanding:

1. Run all code examples without errors
2. Create a simple URDF model and validate it
3. Implement a basic AI agent that sends commands to a ROS 2 node
