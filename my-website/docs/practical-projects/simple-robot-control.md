---
sidebar_position: 2
title: Simple Robot Control
description: Building your first robot controller
---

# Simple Robot Control

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will:
- Implement joint-level robot control
- Create position and velocity controllers
- Execute simple pick-and-place tasks
- Visualize robot motion

</div>

## Joint Controller

```python
import pybullet as p
import numpy as np
import time

class JointController:
    """Control individual robot joints."""
    
    def __init__(self, robot_id: int, joint_indices: list):
        self.robot_id = robot_id
        self.joint_indices = joint_indices
    
    def set_positions(self, positions: list, max_velocity: float = 1.0):
        """Move joints to target positions."""
        for i, pos in zip(self.joint_indices, positions):
            p.setJointMotorControl2(
                self.robot_id, i,
                controlMode=p.POSITION_CONTROL,
                targetPosition=pos,
                maxVelocity=max_velocity,
                force=200
            )
    
    def get_positions(self) -> list:
        """Get current joint positions."""
        positions = []
        for i in self.joint_indices:
            state = p.getJointState(self.robot_id, i)
            positions.append(state[0])
        return positions


# Usage example
def simple_motion_demo(robot_id):
    """Execute simple motion sequence."""
    controller = JointController(robot_id, list(range(7)))
    
    # Home position
    home = [0, 0, 0, -1.57, 0, 1.57, 0]
    controller.set_positions(home)
    
    # Wait for motion
    for _ in range(100):
        p.stepSimulation()
        time.sleep(1/240)
    
    # Move to work position
    work = [0.5, 0.3, 0, -1.2, 0, 1.2, 0]
    controller.set_positions(work)
    
    print("Motion demo complete!")
```

## Pick and Place

```python
class PickAndPlace:
    """Simple pick and place operation."""
    
    def __init__(self, robot_id: int, gripper_id: int):
        self.robot_id = robot_id
        self.gripper_id = gripper_id
        self.controller = JointController(robot_id, list(range(7)))
    
    def execute(self, pick_pos: list, place_pos: list):
        """Execute pick and place sequence."""
        
        # Move above pick
        above_pick = pick_pos.copy()
        above_pick[2] += 0.1
        self.move_to(above_pick)
        
        # Descend and grasp
        self.move_to(pick_pos)
        self.close_gripper()
        
        # Lift
        self.move_to(above_pick)
        
        # Move above place
        above_place = place_pos.copy()
        above_place[2] += 0.1
        self.move_to(above_place)
        
        # Place and release
        self.move_to(place_pos)
        self.open_gripper()
        
        # Retreat
        self.move_to(above_place)
        
        print("Pick and place complete!")
```

<div className="key-takeaways">

#### ✅ Key Takeaways

- **Joint control** is the foundation of robot motion
- **Position control** moves to specific angles
- **Sequences** chain simple motions into tasks
- **Simulation** allows safe experimentation

**Next Chapter**: [AI-Powered Navigation →](/docs/practical-projects/ai-powered-navigation)

</div>



