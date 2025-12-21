---
sidebar_position: 1
title: Project Setup
description: Setting up your practical robotics project environment
---

# Project Setup

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will be able to:
- Set up a complete robotics project structure
- Configure simulation environments
- Establish testing workflows
- Prepare for real robot deployment

</div>

## Project Structure

```
physical_ai_project/
├── src/
│   ├── perception/       # Sensor processing
│   ├── planning/         # Motion and task planning
│   ├── control/          # Robot controllers
│   └── agents/           # AI agent implementations
├── config/
│   ├── robot.yaml        # Robot configuration
│   └── simulation.yaml   # Simulation settings
├── tests/
│   ├── unit/             # Unit tests
│   └── integration/      # Integration tests
├── scripts/
│   ├── launch_sim.py     # Start simulation
│   └── run_demo.py       # Demo scripts
├── requirements.txt
└── README.md
```

## Environment Setup

```python
# setup_project.py
import subprocess
import sys
from pathlib import Path

def setup_environment():
    """Configure the development environment."""
    
    # Create virtual environment
    subprocess.run([sys.executable, "-m", "venv", "venv"])
    
    # Install dependencies
    pip_path = Path("venv/Scripts/pip") if sys.platform == "win32" else Path("venv/bin/pip")
    subprocess.run([str(pip_path), "install", "-r", "requirements.txt"])
    
    print("✓ Environment setup complete!")
    print("Activate with: source venv/bin/activate (Linux/Mac)")
    print("            or: .\\venv\\Scripts\\activate (Windows)")

if __name__ == "__main__":
    setup_environment()
```

## Simulation Launch

```python
# scripts/launch_sim.py
import pybullet as p
import pybullet_data
import time

def launch_simulation():
    """Launch PyBullet simulation environment."""
    
    # Connect to physics server
    physics_client = p.connect(p.GUI)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    p.setGravity(0, 0, -9.81)
    
    # Load environment
    plane_id = p.loadURDF("plane.urdf")
    robot_id = p.loadURDF("kuka_iiwa/model.urdf", [0, 0, 0], useFixedBase=True)
    
    print(f"✓ Simulation launched")
    print(f"  Robot ID: {robot_id}")
    print(f"  Joints: {p.getNumJoints(robot_id)}")
    
    return physics_client, robot_id

if __name__ == "__main__":
    client, robot = launch_simulation()
    
    # Keep running
    while True:
        p.stepSimulation()
        time.sleep(1./240.)
```

<div className="key-takeaways">

#### ✅ Key Takeaways

- **Project structure** organizes code for maintainability
- **Virtual environments** isolate dependencies
- **Simulation** enables safe development and testing
- Consistent setup enables team collaboration

**Next Chapter**: [Simple Robot Control →](/docs/practical-projects/simple-robot-control)

</div>



