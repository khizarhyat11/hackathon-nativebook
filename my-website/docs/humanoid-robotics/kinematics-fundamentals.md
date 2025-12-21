---
sidebar_position: 3
title: Kinematics Fundamentals
description: The mathematics of robot motion and spatial relationships
---

# Kinematics Fundamentals

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will be able to:
- Represent robot poses using transformation matrices
- Calculate forward kinematics for serial manipulators
- Solve inverse kinematics problems
- Apply the Denavit-Hartenberg convention

</div>

## What is Kinematics?

**Kinematics** is the study of motion without considering forces. In robotics, kinematics answers two fundamental questions:

- **Forward Kinematics (FK)**: Given joint angles, where is the end-effector?
- **Inverse Kinematics (IK)**: Given a desired end-effector pose, what joint angles achieve it?

```
Forward Kinematics (FK)
    Joint Angles → End-Effector Pose
    [θ₁, θ₂, θ₃, ...] → [x, y, z, roll, pitch, yaw]

Inverse Kinematics (IK)
    End-Effector Pose → Joint Angles
    [x, y, z, roll, pitch, yaw] → [θ₁, θ₂, θ₃, ...]
```

## Spatial Representations

### Position and Orientation

```python
import numpy as np
from dataclasses import dataclass
from typing import Tuple

@dataclass
class Position:
    """3D position in space."""
    x: float
    y: float
    z: float
    
    def as_array(self) -> np.ndarray:
        return np.array([self.x, self.y, self.z])
    
    def distance_to(self, other: 'Position') -> float:
        return np.linalg.norm(self.as_array() - other.as_array())


@dataclass
class Orientation:
    """3D orientation using roll-pitch-yaw (Euler angles)."""
    roll: float   # Rotation around X
    pitch: float  # Rotation around Y  
    yaw: float    # Rotation around Z
    
    def to_rotation_matrix(self) -> np.ndarray:
        """Convert Euler angles to 3x3 rotation matrix."""
        c_r, s_r = np.cos(self.roll), np.sin(self.roll)
        c_p, s_p = np.cos(self.pitch), np.sin(self.pitch)
        c_y, s_y = np.cos(self.yaw), np.sin(self.yaw)
        
        # ZYX Euler angles (yaw-pitch-roll)
        R = np.array([
            [c_y*c_p, c_y*s_p*s_r - s_y*c_r, c_y*s_p*c_r + s_y*s_r],
            [s_y*c_p, s_y*s_p*s_r + c_y*c_r, s_y*s_p*c_r - c_y*s_r],
            [-s_p,    c_p*s_r,               c_p*c_r]
        ])
        return R


@dataclass
class Pose:
    """Complete 6-DOF pose: position + orientation."""
    position: Position
    orientation: Orientation
    
    def to_matrix(self) -> np.ndarray:
        """Convert to 4x4 homogeneous transformation matrix."""
        T = np.eye(4)
        T[:3, :3] = self.orientation.to_rotation_matrix()
        T[:3, 3] = self.position.as_array()
        return T
```

### Homogeneous Transformation Matrices

The 4x4 homogeneous transformation matrix is the workhorse of robot kinematics:

```python
class TransformationMatrix:
    """4x4 homogeneous transformation matrix operations."""
    
    def __init__(self, matrix: np.ndarray = None):
        if matrix is None:
            self.matrix = np.eye(4)
        else:
            self.matrix = matrix
    
    @property
    def rotation(self) -> np.ndarray:
        """Extract 3x3 rotation matrix."""
        return self.matrix[:3, :3]
    
    @property
    def translation(self) -> np.ndarray:
        """Extract translation vector."""
        return self.matrix[:3, 3]
    
    @classmethod
    def from_rotation_x(cls, angle: float) -> 'TransformationMatrix':
        """Rotation around X-axis."""
        c, s = np.cos(angle), np.sin(angle)
        T = np.eye(4)
        T[1, 1], T[1, 2] = c, -s
        T[2, 1], T[2, 2] = s, c
        return cls(T)
    
    @classmethod
    def from_rotation_z(cls, angle: float) -> 'TransformationMatrix':
        """Rotation around Z-axis."""
        c, s = np.cos(angle), np.sin(angle)
        T = np.eye(4)
        T[0, 0], T[0, 1] = c, -s
        T[1, 0], T[1, 1] = s, c
        return cls(T)
    
    @classmethod
    def from_translation(cls, x: float, y: float, z: float) -> 'TransformationMatrix':
        """Pure translation."""
        T = np.eye(4)
        T[0, 3], T[1, 3], T[2, 3] = x, y, z
        return cls(T)
    
    def __matmul__(self, other: 'TransformationMatrix') -> 'TransformationMatrix':
        """Matrix multiplication: T1 @ T2."""
        return TransformationMatrix(self.matrix @ other.matrix)
    
    def inverse(self) -> 'TransformationMatrix':
        """Compute inverse transformation."""
        R = self.rotation
        t = self.translation
        
        T_inv = np.eye(4)
        T_inv[:3, :3] = R.T
        T_inv[:3, 3] = -R.T @ t
        
        return TransformationMatrix(T_inv)


# Example: Chain of transformations
T_base_to_link1 = TransformationMatrix.from_rotation_z(np.pi/4)
T_link1_to_link2 = TransformationMatrix.from_translation(0.5, 0, 0)
T_link2_to_end = TransformationMatrix.from_rotation_z(np.pi/4)

# End-effector pose relative to base
T_base_to_end = T_base_to_link1 @ T_link1_to_link2 @ T_link2_to_end
print("End-effector position:", T_base_to_end.translation)
```

## Denavit-Hartenberg Convention

The DH convention provides a systematic way to describe robot geometry:

```python
from dataclasses import dataclass
from typing import List

@dataclass
class DHParameter:
    """Denavit-Hartenberg parameters for one joint."""
    
    a: float      # Link length (along X)
    alpha: float  # Link twist (around X)
    d: float      # Link offset (along Z)
    theta: float  # Joint angle (around Z) - variable for revolute
    
    def to_matrix(self, joint_value: float = None) -> np.ndarray:
        """
        Compute transformation matrix for this DH frame.
        
        For revolute joints: joint_value is theta
        For prismatic joints: joint_value is d
        """
        if joint_value is not None:
            theta = joint_value
        else:
            theta = self.theta
        
        ct, st = np.cos(theta), np.sin(theta)
        ca, sa = np.cos(self.alpha), np.sin(self.alpha)
        
        T = np.array([
            [ct, -st*ca,  st*sa, self.a*ct],
            [st,  ct*ca, -ct*sa, self.a*st],
            [0,   sa,     ca,    self.d],
            [0,   0,      0,     1]
        ])
        
        return T


class DHRobot:
    """Robot defined by DH parameters."""
    
    def __init__(self, dh_params: List[DHParameter]):
        self.dh_params = dh_params
        self.num_joints = len(dh_params)
    
    def forward_kinematics(self, joint_angles: List[float]) -> np.ndarray:
        """
        Compute end-effector pose given joint angles.
        
        Returns 4x4 transformation matrix from base to end-effector.
        """
        if len(joint_angles) != self.num_joints:
            raise ValueError(f"Expected {self.num_joints} joint angles")
        
        T = np.eye(4)
        
        for i, (dh, theta) in enumerate(zip(self.dh_params, joint_angles)):
            T_i = dh.to_matrix(theta)
            T = T @ T_i
        
        return T
    
    def get_joint_positions(self, joint_angles: List[float]) -> List[np.ndarray]:
        """Get position of each joint for visualization."""
        
        positions = [np.array([0, 0, 0])]  # Base
        T = np.eye(4)
        
        for dh, theta in zip(self.dh_params, joint_angles):
            T = T @ dh.to_matrix(theta)
            positions.append(T[:3, 3].copy())
        
        return positions


# Example: 2-DOF planar arm
planar_arm = DHRobot([
    DHParameter(a=1.0, alpha=0, d=0, theta=0),  # Link 1: 1m
    DHParameter(a=0.8, alpha=0, d=0, theta=0),  # Link 2: 0.8m
])

# Forward kinematics
joint_angles = [np.pi/4, np.pi/3]  # 45° and 60°
T_end = planar_arm.forward_kinematics(joint_angles)
print(f"End-effector position: {T_end[:3, 3]}")
```

## Inverse Kinematics

Computing joint angles from desired end-effector pose:

```python
class InverseKinematics:
    """Inverse kinematics solvers."""
    
    @staticmethod
    def analytical_2dof_planar(target: Tuple[float, float], 
                                L1: float, L2: float) -> Tuple[float, float]:
        """
        Analytical IK for 2-DOF planar arm.
        
        Returns (theta1, theta2) or None if unreachable.
        """
        x, y = target
        
        # Check reachability
        distance = np.sqrt(x**2 + y**2)
        if distance > L1 + L2 or distance < abs(L1 - L2):
            return None  # Unreachable
        
        # Law of cosines for theta2
        cos_theta2 = (x**2 + y**2 - L1**2 - L2**2) / (2 * L1 * L2)
        cos_theta2 = np.clip(cos_theta2, -1, 1)
        
        theta2 = np.arccos(cos_theta2)  # Elbow up solution
        # theta2 = -np.arccos(cos_theta2)  # Elbow down solution
        
        # Theta1
        k1 = L1 + L2 * np.cos(theta2)
        k2 = L2 * np.sin(theta2)
        theta1 = np.arctan2(y, x) - np.arctan2(k2, k1)
        
        return (theta1, theta2)
    
    @staticmethod
    def jacobian_ik(robot: DHRobot, 
                    target_pose: np.ndarray,
                    initial_guess: List[float],
                    max_iterations: int = 100,
                    tolerance: float = 1e-3) -> List[float]:
        """
        Numerical IK using Jacobian pseudo-inverse.
        
        This is the general method that works for any robot.
        """
        q = np.array(initial_guess)
        
        for iteration in range(max_iterations):
            # Current end-effector pose
            T_current = robot.forward_kinematics(q)
            
            # Position error
            pos_error = target_pose[:3, 3] - T_current[:3, 3]
            
            # Check convergence
            if np.linalg.norm(pos_error) < tolerance:
                return q.tolist()
            
            # Compute Jacobian numerically
            J = InverseKinematics._numerical_jacobian(robot, q)
            
            # Pseudo-inverse for joint velocity
            J_pinv = np.linalg.pinv(J)
            dq = J_pinv @ pos_error
            
            # Update joints
            q = q + 0.5 * dq  # Step size of 0.5
        
        return q.tolist()  # Return best effort
    
    @staticmethod
    def _numerical_jacobian(robot: DHRobot, q: np.ndarray, 
                            delta: float = 1e-6) -> np.ndarray:
        """Compute Jacobian numerically via finite differences."""
        
        n = len(q)
        J = np.zeros((3, n))  # Position Jacobian only
        
        T0 = robot.forward_kinematics(q)
        p0 = T0[:3, 3]
        
        for i in range(n):
            q_perturb = q.copy()
            q_perturb[i] += delta
            
            T_perturb = robot.forward_kinematics(q_perturb)
            p_perturb = T_perturb[:3, 3]
            
            J[:, i] = (p_perturb - p0) / delta
        
        return J


# Example usage
L1, L2 = 1.0, 0.8
target = (1.2, 0.5)

solution = InverseKinematics.analytical_2dof_planar(target, L1, L2)
if solution:
    print(f"Joint angles: θ1={np.degrees(solution[0]):.1f}°, θ2={np.degrees(solution[1]):.1f}°")
else:
    print("Target unreachable")
```

## Workspace Analysis

Understanding where a robot can reach:

```python
class WorkspaceAnalyzer:
    """Analyze robot workspace characteristics."""
    
    def __init__(self, robot: DHRobot, joint_limits: List[Tuple[float, float]]):
        self.robot = robot
        self.joint_limits = joint_limits
    
    def sample_workspace(self, samples_per_joint: int = 20) -> np.ndarray:
        """Sample reachable positions in the workspace."""
        
        positions = []
        
        # Create grid of joint angles
        joint_ranges = [
            np.linspace(limits[0], limits[1], samples_per_joint)
            for limits in self.joint_limits
        ]
        
        # Sample all combinations (exponential!)
        from itertools import product
        for joint_config in product(*joint_ranges):
            T = self.robot.forward_kinematics(list(joint_config))
            positions.append(T[:3, 3])
        
        return np.array(positions)
    
    def workspace_bounds(self, positions: np.ndarray) -> dict:
        """Calculate workspace bounds."""
        return {
            'x_min': positions[:, 0].min(),
            'x_max': positions[:, 0].max(),
            'y_min': positions[:, 1].min(),
            'y_max': positions[:, 1].max(),
            'z_min': positions[:, 2].min(),
            'z_max': positions[:, 2].max(),
            'max_reach': np.linalg.norm(positions, axis=1).max(),
        }

    def is_reachable(self, target: np.ndarray) -> bool:
        """Check if a target position is potentially reachable."""
        bounds = self.workspace_bounds(self.sample_workspace(10))
        
        if target[0] < bounds['x_min'] or target[0] > bounds['x_max']:
            return False
        if target[1] < bounds['y_min'] or target[1] > bounds['y_max']:
            return False
        if target[2] < bounds['z_min'] or target[2] > bounds['z_max']:
            return False
        
        return True
```

## Summary

Kinematics provides the mathematical foundation for robot motion:

- **Transformation matrices** represent poses and chain together
- **Forward kinematics** maps joint angles to end-effector pose
- **Inverse kinematics** solves for joint angles given a target pose
- **DH parameters** provide a systematic way to describe robot geometry

<div className="key-takeaways">

#### ✅ Key Takeaways

- **4x4 homogeneous matrices** combine rotation and translation
- **DH convention** systematically describes serial robot geometry
- **FK is unique**; **IK may have multiple solutions** or none
- **Jacobian-based IK** provides a general numerical solution

**Next Chapter**: [Motion Planning →](/docs/humanoid-robotics/motion-planning)

</div>



