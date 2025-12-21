---
sidebar_position: 4
title: Development Environment
description: Setting up your Physical AI development environment
---

# Development Environment Setup

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will be able to:
- Set up a complete Physical AI development environment
- Install and configure Python, ROS2, and simulation tools
- Understand virtual environments and dependency management
- Run your first robot simulation

</div>

## Overview

A well-configured development environment is essential for Physical AI work. We'll set up:

- **Python** - Primary programming language
- **ROS2** - Robot Operating System 2
- **Simulation** - Virtual robots for testing
- **IDE** - Code editor with robotics support

```
┌─────────────────────────────────────────────────────────────────┐
│                 DEVELOPMENT ENVIRONMENT                         │
├─────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐             │
│  │    IDE      │  │  Terminal   │  │ Simulation  │             │
│  │  (VS Code)  │  │  (ROS2)     │  │  (Gazebo)   │             │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘             │
│         │                │                │                     │
│  ┌──────▼────────────────▼────────────────▼──────┐             │
│  │               Python Environment              │             │
│  │  ┌────────────────────────────────────────┐  │             │
│  │  │ numpy, scipy, matplotlib, opencv       │  │             │
│  │  │ torch, tensorflow, transformers        │  │             │
│  │  │ rclpy, sensor_msgs, geometry_msgs      │  │             │
│  │  └────────────────────────────────────────┘  │             │
│  └───────────────────────────────────────────────┘             │
│                            │                                    │
│  ┌─────────────────────────▼─────────────────────┐             │
│  │              Operating System                 │             │
│  │          (Ubuntu 22.04 recommended)           │             │
│  └───────────────────────────────────────────────┘             │
└─────────────────────────────────────────────────────────────────┘
```

## Operating System Recommendations

| Platform | Recommendation | Notes |
|----------|---------------|-------|
| **Ubuntu 22.04** | ⭐ Recommended | Best ROS2 support |
| **Windows 11** | Good | Use WSL2 for ROS2 |
| **macOS** | Limited | ROS2 partial support |
| **Docker** | Alternative | Containerized environment |

:::tip For Windows Users
Use Windows Subsystem for Linux (WSL2) with Ubuntu 22.04 for the best experience. This gives you native Linux tools while staying on Windows.
:::

## Python Environment Setup

### Installing Python

```bash
# Ubuntu/WSL2
sudo apt update
sudo apt install python3.10 python3.10-venv python3-pip

# Verify installation
python3 --version  # Should show 3.10.x
pip3 --version
```

### Creating a Virtual Environment

Always use virtual environments to isolate project dependencies:

```bash
# Create project directory
mkdir ~/physical_ai_workspace
cd ~/physical_ai_workspace

# Create virtual environment
python3 -m venv venv

# Activate (do this every time you work on the project)
source venv/bin/activate  # Linux/Mac
# or
.\venv\Scripts\activate   # Windows

# Your prompt should now show (venv)
```

### Installing Core Packages

```bash
# Core scientific computing
pip install numpy scipy matplotlib

# Computer vision
pip install opencv-python pillow

# Machine learning
pip install torch torchvision
pip install transformers

# Robotics utilities
pip install spatialmath-python
pip install roboticstoolbox-python

# Development tools
pip install jupyter ipython
pip install pytest black flake8
```

Create a `requirements.txt` for reproducibility:

```txt
# requirements.txt
numpy>=1.24.0
scipy>=1.10.0
matplotlib>=3.7.0
opencv-python>=4.8.0
torch>=2.0.0
torchvision>=0.15.0
transformers>=4.30.0
spatialmath-python>=1.0.0
roboticstoolbox-python>=1.0.0
jupyter>=1.0.0
pytest>=7.0.0
black>=23.0.0
```

```bash
# Install from requirements file
pip install -r requirements.txt
```

## ROS2 Installation

### Ubuntu 22.04 Native Installation

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8

# Add ROS2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble (LTS)
sudo apt update
sudo apt install ros-humble-desktop

# Install development tools
sudo apt install ros-dev-tools

# Source ROS2 in your shell
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Verify installation
ros2 --help
```

### Windows WSL2 Installation

```bash
# First, ensure WSL2 Ubuntu 22.04 is installed
# Then follow the Ubuntu installation steps above

# For GUI applications (Gazebo, RViz), install VcXsrv or WSLg
```

### Creating a ROS2 Workspace

```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# Build (even empty workspace)
colcon build

# Source the workspace
source install/setup.bash

# Add to bashrc for convenience
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## Simulation Environment

### Installing Gazebo

```bash
# Gazebo is included with ros-humble-desktop, but if needed:
sudo apt install ros-humble-gazebo-ros-pkgs

# Install additional simulation packages
sudo apt install ros-humble-robot-state-publisher
sudo apt install ros-humble-joint-state-publisher-gui
sudo apt install ros-humble-xacro
```

### PyBullet (Lightweight Alternative)

```python
# Install PyBullet
pip install pybullet

# Quick test
import pybullet as p
import pybullet_data
import time

# Start simulation
physics_client = p.connect(p.GUI)
p.setAdditionalSearchPath(pybullet_data.getDataPath())

# Load ground and robot
plane_id = p.loadURDF("plane.urdf")
robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0])

# Set gravity
p.setGravity(0, 0, -10)

# Run simulation
for i in range(10000):
    p.stepSimulation()
    time.sleep(1./240.)

p.disconnect()
```

### NVIDIA Isaac Sim (Advanced)

For high-fidelity simulation with GPU acceleration:

```bash
# Requires NVIDIA GPU and Omniverse Launcher
# Download from: https://developer.nvidia.com/isaac-sim

# Isaac Sim provides:
# - Photorealistic rendering
# - Accurate physics
# - Domain randomization
# - ROS2 integration
```

## IDE Setup

### VS Code Configuration

1. **Install VS Code**: https://code.visualstudio.com/

2. **Install Extensions**:
```
Python
Pylance
ROS
URDF
C/C++ (for ROS2 C++ nodes)
Jupyter
```

3. **Configure settings** (`settings.json`):
```json
{
    "python.defaultInterpreterPath": "~/physical_ai_workspace/venv/bin/python",
    "python.formatting.provider": "black",
    "python.linting.enabled": true,
    "python.linting.flake8Enabled": true,
    "editor.formatOnSave": true,
    "files.associations": {
        "*.urdf": "xml",
        "*.xacro": "xml",
        "*.sdf": "xml"
    }
}
```

4. **ROS2 Extension Setup**:
```json
// In settings.json
{
    "ros.distro": "humble"
}
```

## Your First Simulation

Let's verify everything works by running a simple simulation:

```python
# first_simulation.py
"""
Your first Physical AI simulation!
A simple robot arm reaching for a target.
"""

import pybullet as p
import pybullet_data
import numpy as np
import time

def setup_simulation():
    """Initialize PyBullet simulation."""
    # Connect to physics server with GUI
    physics_client = p.connect(p.GUI)
    
    # Add PyBullet's built-in models path
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    
    # Set gravity
    p.setGravity(0, 0, -9.81)
    
    # Load ground plane
    plane_id = p.loadURDF("plane.urdf")
    
    return physics_client


def load_robot():
    """Load a robot arm."""
    # Load Kuka IIWA robot arm
    robot_id = p.loadURDF(
        "kuka_iiwa/model.urdf",
        basePosition=[0, 0, 0],
        useFixedBase=True
    )
    
    # Get joint info
    num_joints = p.getNumJoints(robot_id)
    print(f"Robot has {num_joints} joints")
    
    for i in range(num_joints):
        info = p.getJointInfo(robot_id, i)
        print(f"  Joint {i}: {info[1].decode('utf-8')}, type={info[2]}")
    
    return robot_id


def set_joint_positions(robot_id, positions):
    """Set robot joint positions."""
    for i, pos in enumerate(positions):
        p.setJointMotorControl2(
            robot_id,
            jointIndex=i,
            controlMode=p.POSITION_CONTROL,
            targetPosition=pos,
            force=500  # Max force
        )


def get_end_effector_position(robot_id):
    """Get the position of the robot's end effector."""
    # Joint 6 is the end effector link for Kuka
    state = p.getLinkState(robot_id, 6)
    return state[0]  # World position


def create_target(position):
    """Create a visual target sphere."""
    visual_shape = p.createVisualShape(
        p.GEOM_SPHERE,
        radius=0.05,
        rgbaColor=[1, 0, 0, 0.8]  # Red
    )
    target_id = p.createMultiBody(
        baseVisualShapeIndex=visual_shape,
        basePosition=position
    )
    return target_id


def simple_ik_controller(robot_id, target_pos, num_iterations=100):
    """
    Simple iterative IK controller.
    Moves end effector toward target.
    """
    for i in range(num_iterations):
        current_pos = get_end_effector_position(robot_id)
        
        # Calculate error
        error = np.array(target_pos) - np.array(current_pos)
        distance = np.linalg.norm(error)
        
        if distance < 0.01:
            print(f"Reached target! Distance: {distance:.4f}m")
            break
        
        # Use PyBullet's built-in IK
        joint_positions = p.calculateInverseKinematics(
            robot_id,
            endEffectorLinkIndex=6,
            targetPosition=target_pos
        )
        
        # Apply joint positions
        set_joint_positions(robot_id, joint_positions[:7])
        
        # Step simulation
        for _ in range(10):  # Multiple physics steps
            p.stepSimulation()
            time.sleep(1./240.)
        
        if i % 10 == 0:
            print(f"Iteration {i}: distance to target = {distance:.4f}m")


def main():
    """Main simulation loop."""
    print("=" * 50)
    print("Physical AI - First Simulation")
    print("=" * 50)
    
    # Setup
    physics_client = setup_simulation()
    robot_id = load_robot()
    
    # Create target
    target_position = [0.5, 0.3, 0.5]  # x, y, z
    target_id = create_target(target_position)
    print(f"\nTarget position: {target_position}")
    
    # Move robot home first
    home_position = [0, 0, 0, -1.57, 0, 1.57, 0]
    set_joint_positions(robot_id, home_position)
    
    # Let robot settle
    print("\nMoving to home position...")
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1./240.)
    
    # Move to target
    print("\nReaching for target...")
    simple_ik_controller(robot_id, target_position)
    
    # Keep simulation running
    print("\nSimulation complete! Press Ctrl+C to exit.")
    try:
        while True:
            p.stepSimulation()
            time.sleep(1./240.)
    except KeyboardInterrupt:
        pass
    
    p.disconnect()
    print("Simulation ended.")


if __name__ == "__main__":
    main()
```

Run the simulation:

```bash
# Activate your virtual environment first
source ~/physical_ai_workspace/venv/bin/activate

# Run the simulation
python first_simulation.py
```

## Environment Verification Checklist

Run these checks to verify your setup:

```python
# verify_environment.py
"""Verify Physical AI development environment."""

import sys

def check_python():
    print(f"Python: {sys.version}")
    assert sys.version_info >= (3, 10), "Python 3.10+ required"
    print("  ✓ Python version OK")

def check_packages():
    packages = [
        "numpy",
        "scipy", 
        "matplotlib",
        "cv2",  # opencv
        "torch",
        "pybullet",
    ]
    
    for pkg in packages:
        try:
            __import__(pkg)
            print(f"  ✓ {pkg}")
        except ImportError:
            print(f"  ✗ {pkg} - MISSING!")

def check_ros2():
    try:
        import rclpy
        print("  ✓ ROS2 (rclpy)")
    except ImportError:
        print("  ✗ ROS2 - Not installed or not sourced")
        print("    Run: source /opt/ros/humble/setup.bash")

def main():
    print("=" * 50)
    print("Physical AI Environment Check")
    print("=" * 50)
    
    print("\n1. Python Version:")
    check_python()
    
    print("\n2. Core Packages:")
    check_packages()
    
    print("\n3. ROS2:")
    check_ros2()
    
    print("\n" + "=" * 50)
    print("Environment check complete!")
    print("=" * 50)

if __name__ == "__main__":
    main()
```

## Summary

You now have a complete Physical AI development environment:

- Python with scientific computing and ML packages
- ROS2 for robotics middleware
- Simulation tools (PyBullet and/or Gazebo)
- VS Code configured for robotics development

<div className="key-takeaways">

#### ✅ Key Takeaways

- Use **virtual environments** to isolate project dependencies
- **ROS2 Humble** is the recommended robotics middleware
- **PyBullet** provides a quick-start simulation environment
- **Verify your setup** before starting development

**Next Part**: [Part 2: Humanoid Robotics →](/docs/category/digital-twin)

</div>



