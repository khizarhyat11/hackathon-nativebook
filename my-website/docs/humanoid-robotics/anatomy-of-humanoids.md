---
sidebar_position: 1
title: Anatomy of Humanoid Robots
description: Understanding the structure and design of human-like robots
---

# Anatomy of Humanoid Robots

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will be able to:
- Identify the major components of humanoid robots
- Understand degrees of freedom and joint configurations
- Compare different humanoid robot architectures
- Explain the design tradeoffs in humanoid construction

</div>

## Why Humanoid Form?

Humanoid robots are designed to mirror human anatomy. This design choice is driven by several factors:

```
┌─────────────────────────────────────────────────────────────────┐
│               WHY HUMANOID DESIGN?                              │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│   🏠 Human Environments        🤝 Human Interaction             │
│   ├── Stairs, doors, tools    ├── Intuitive communication      │
│   ├── Furniture, workspaces   ├── Social acceptance            │
│   └── Vehicles, equipment     └── Non-verbal cues              │
│                                                                 │
│   🔧 Human Tools              🧠 Transfer Learning              │
│   ├── Designed for hands      ├── Human demonstrations         │
│   ├── Bipedal access          ├── Motion capture data          │
│   └── Manipulation reach      └── Teleoperation                │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘
```

## Humanoid Robot Structure

### Overall Kinematic Chain

```python
from dataclasses import dataclass
from typing import List, Tuple
from enum import Enum

class JointType(Enum):
    REVOLUTE = "revolute"      # Rotation around axis
    PRISMATIC = "prismatic"    # Linear translation
    SPHERICAL = "spherical"    # Ball joint (3 DOF)
    FIXED = "fixed"            # No motion


@dataclass
class Link:
    """A rigid body segment of the robot."""
    name: str
    length: float        # meters
    mass: float          # kg
    center_of_mass: Tuple[float, float, float]
    inertia_tensor: List[List[float]]


@dataclass
class Joint:
    """Connection between two links."""
    name: str
    joint_type: JointType
    parent_link: str
    child_link: str
    axis: Tuple[float, float, float]  # Rotation/translation axis
    limits: Tuple[float, float]        # min, max position
    max_velocity: float                 # rad/s or m/s
    max_torque: float                   # Nm or N


@dataclass
class HumanoidStructure:
    """Complete humanoid robot definition."""
    
    # Core body
    pelvis: Link
    torso: Link
    chest: Link
    head: Link
    
    # Arms (symmetric)
    shoulder: Joint      # 3 DOF typically
    upper_arm: Link
    elbow: Joint         # 1-2 DOF
    forearm: Link
    wrist: Joint         # 2-3 DOF
    hand: Link           # Many DOF for fingers
    
    # Legs (symmetric)
    hip: Joint           # 3 DOF typically
    thigh: Link
    knee: Joint          # 1 DOF
    shin: Link
    ankle: Joint         # 2 DOF
    foot: Link
    
    @property
    def total_dof(self) -> int:
        """Calculate total degrees of freedom."""
        # Typical humanoid: 30-50+ DOF
        body_dof = 6  # Floating base
        arm_dof = 7 * 2  # 7 DOF per arm
        leg_dof = 6 * 2  # 6 DOF per leg
        hand_dof = 12 * 2  # Simplified hands
        head_dof = 2  # Pan/tilt
        return body_dof + arm_dof + leg_dof + hand_dof + head_dof
```

### Degrees of Freedom Reference

| Body Part | Typical DOF | Human DOF | Function |
|-----------|-------------|-----------|----------|
| **Neck/Head** | 2-3 | 7 | Gaze direction, sensing |
| **Torso** | 1-3 | 30+ | Flexibility, reach extension |
| **Shoulder** | 3 | 3 | Arm positioning |
| **Elbow** | 1-2 | 2 | Reach, manipulation |
| **Wrist** | 2-3 | 3 | End-effector orientation |
| **Hand/Fingers** | 10-24 | 25+ | Grasping, manipulation |
| **Hip** | 3 | 3 | Leg positioning, balance |
| **Knee** | 1 | 1 | Leg extension |
| **Ankle** | 2 | 2 | Balance, ground contact |

## Head and Sensing Systems

The head houses critical sensors for perception:

```python
@dataclass
class HumanoidHead:
    """Head assembly with sensors."""
    
    # Vision system
    cameras: List['Camera']
    camera_configuration: str  # "stereo", "rgbd", "array"
    
    # Audio
    microphones: List['Microphone']
    microphone_array: bool  # For sound localization
    
    # Additional sensors
    imu: 'IMU'  # Head orientation
    lidar: 'LiDAR' = None  # Optional 3D scanning
    
    # Actuation
    neck_pan_joint: Joint
    neck_tilt_joint: Joint
    eye_cameras_independent: bool = False  # Can eyes move independently?


@dataclass
class Camera:
    """Vision sensor specifications."""
    resolution: Tuple[int, int]  # pixels
    field_of_view: float         # degrees
    frame_rate: int              # Hz
    sensor_type: str             # "RGB", "RGBD", "Event"


# Example: Research humanoid head configuration
research_head = HumanoidHead(
    cameras=[
        Camera((1920, 1080), 70.0, 30, "RGB"),  # Left eye
        Camera((1920, 1080), 70.0, 30, "RGB"),  # Right eye
    ],
    camera_configuration="stereo",
    microphones=[],  # 8 microphones
    microphone_array=True,
    imu=None,  # 9-axis IMU
    neck_pan_joint=Joint("neck_pan", JointType.REVOLUTE, 
                         "chest", "head", (0, 0, 1), (-1.5, 1.5), 2.0, 10.0),
    neck_tilt_joint=Joint("neck_tilt", JointType.REVOLUTE,
                          "head_pan_link", "head", (0, 1, 0), (-0.5, 0.8), 2.0, 10.0),
)
```

## Arms and Manipulation

Human-like arms for dexterous manipulation:

```python
@dataclass
class HumanoidArm:
    """7-DOF anthropomorphic arm."""
    
    # Shoulder complex (3 DOF)
    shoulder_pitch: Joint  # Forward/backward
    shoulder_roll: Joint   # In/out
    shoulder_yaw: Joint    # Rotation
    
    # Elbow (1-2 DOF)
    elbow_pitch: Joint     # Flexion/extension
    elbow_roll: Joint      # Forearm rotation (optional)
    
    # Wrist (2-3 DOF)
    wrist_pitch: Joint
    wrist_roll: Joint
    wrist_yaw: Joint = None  # Optional
    
    # Link properties
    upper_arm_length: float = 0.28  # meters
    forearm_length: float = 0.25    # meters
    
    def workspace_volume(self) -> float:
        """Approximate reachable workspace volume."""
        max_reach = self.upper_arm_length + self.forearm_length
        # Approximate as sphere fraction
        import math
        return (4/3) * math.pi * (max_reach ** 3) * 0.6  # ~60% coverage


def create_standard_arm(side: str) -> HumanoidArm:
    """Create a standard 7-DOF humanoid arm."""
    
    sign = 1 if side == "left" else -1
    
    return HumanoidArm(
        shoulder_pitch=Joint(
            f"{side}_shoulder_pitch", JointType.REVOLUTE,
            "chest", f"{side}_upper_arm",
            axis=(0, 1, 0),
            limits=(-2.0, 2.0),
            max_velocity=2.0,
            max_torque=80.0
        ),
        shoulder_roll=Joint(
            f"{side}_shoulder_roll", JointType.REVOLUTE,
            f"{side}_upper_arm", f"{side}_upper_arm_roll",
            axis=(1, 0, 0),
            limits=(-1.5 * sign, 0.5 * sign),
            max_velocity=2.0,
            max_torque=80.0
        ),
        shoulder_yaw=Joint(
            f"{side}_shoulder_yaw", JointType.REVOLUTE,
            f"{side}_upper_arm_roll", f"{side}_elbow_link",
            axis=(0, 0, 1),
            limits=(-1.7, 1.7),
            max_velocity=2.0,
            max_torque=60.0
        ),
        elbow_pitch=Joint(
            f"{side}_elbow_pitch", JointType.REVOLUTE,
            f"{side}_elbow_link", f"{side}_forearm",
            axis=(0, 1, 0),
            limits=(0.0, 2.5),  # Can't hyperextend
            max_velocity=2.0,
            max_torque=60.0
        ),
        elbow_roll=Joint(
            f"{side}_elbow_roll", JointType.REVOLUTE,
            f"{side}_forearm", f"{side}_wrist_link",
            axis=(1, 0, 0),
            limits=(-1.5, 1.5),
            max_velocity=3.0,
            max_torque=30.0
        ),
        wrist_pitch=Joint(
            f"{side}_wrist_pitch", JointType.REVOLUTE,
            f"{side}_wrist_link", f"{side}_hand",
            axis=(0, 1, 0),
            limits=(-1.0, 1.0),
            max_velocity=3.0,
            max_torque=20.0
        ),
        wrist_roll=Joint(
            f"{side}_wrist_roll", JointType.REVOLUTE,
            f"{side}_hand", f"{side}_palm",
            axis=(1, 0, 0),
            limits=(-1.5, 1.5),
            max_velocity=3.0,
            max_torque=20.0
        ),
    )
```

## Legs and Locomotion

Bipedal legs present unique challenges:

```python
@dataclass
class HumanoidLeg:
    """6-DOF leg for bipedal locomotion."""
    
    # Hip (3 DOF)
    hip_roll: Joint    # Lateral movement
    hip_pitch: Joint   # Forward/backward
    hip_yaw: Joint     # Rotation
    
    # Knee (1 DOF)
    knee_pitch: Joint  # Flexion/extension
    
    # Ankle (2 DOF)
    ankle_pitch: Joint  # Toe up/down
    ankle_roll: Joint   # Lateral tilt
    
    # Dimensions
    thigh_length: float = 0.40   # meters
    shin_length: float = 0.38    # meters
    foot_length: float = 0.25    # meters
    
    def compute_leg_length(self, joint_angles: dict) -> float:
        """Compute effective leg length for given joint configuration."""
        import math
        
        knee_angle = joint_angles.get('knee_pitch', 0)
        # Simplified: straight leg length minus knee bend effect
        straight_length = self.thigh_length + self.shin_length
        bent_reduction = (1 - math.cos(knee_angle)) * self.shin_length
        
        return straight_length - bent_reduction


@dataclass
class BipedalLocomotion:
    """Properties relevant to bipedal walking."""
    
    left_leg: HumanoidLeg
    right_leg: HumanoidLeg
    
    hip_width: float = 0.20  # meters
    total_mass: float = 70.0  # kg
    com_height: float = 1.0   # meters (standing)
    
    def natural_frequency(self) -> float:
        """Natural pendulum frequency for walking."""
        import math
        g = 9.81  # m/s²
        # Simplified inverted pendulum model
        return math.sqrt(g / self.com_height)
    
    def preferred_walking_speed(self) -> float:
        """Estimated comfortable walking speed."""
        # Based on Froude number ≈ 0.25 for comfortable walk
        import math
        g = 9.81
        froude = 0.25
        return math.sqrt(froude * g * self.com_height)
```

## Hands and Grasping

Dexterous hands are among the most complex subsystems:

```python
@dataclass
class FingerJoint:
    """Single finger joint."""
    name: str
    position_range: Tuple[float, float]
    max_force: float


@dataclass  
class Finger:
    """Multi-joint finger."""
    name: str
    joints: List[FingerJoint]
    
    # Finger types
    THUMB = "thumb"      # Opposable, 4 DOF typically
    INDEX = "index"      # 3-4 DOF
    MIDDLE = "middle"    # 3-4 DOF
    RING = "ring"        # 3-4 DOF (often coupled)
    PINKY = "pinky"      # 3-4 DOF (often coupled)


@dataclass
class HumanoidHand:
    """Dexterous robotic hand."""
    
    fingers: List[Finger]
    palm_sensors: bool = True          # Tactile sensing
    finger_tip_sensors: bool = True    # Fine touch
    
    # Actuation method
    actuation: str = "tendon"  # "tendon", "direct", "hydraulic"
    
    @property
    def total_dof(self) -> int:
        return sum(len(f.joints) for f in self.fingers)
    
    def grasp_types_supported(self) -> List[str]:
        """List of grasp primitives this hand can perform."""
        dof = self.total_dof
        
        grasps = ["power_grasp"]  # All hands can do power grasp
        
        if dof >= 10:
            grasps.append("precision_pinch")
            grasps.append("lateral_pinch")
        
        if dof >= 15:
            grasps.append("tripod_grasp")
            grasps.append("cylindrical_grasp")
        
        if dof >= 20:
            grasps.append("spherical_grasp")
            grasps.append("hook_grasp")
            grasps.append("writing_grasp")
        
        return grasps


# Example: Simplified dexterous hand
def create_dexterous_hand() -> HumanoidHand:
    """Create a research-grade dexterous hand."""
    
    fingers = []
    
    # Thumb: 4 DOF (opposition + 3 flexion)
    thumb = Finger(
        name="thumb",
        joints=[
            FingerJoint("thumb_opposition", (-0.5, 1.5), 5.0),
            FingerJoint("thumb_mcp", (0, 1.5), 5.0),
            FingerJoint("thumb_pip", (0, 1.2), 3.0),
            FingerJoint("thumb_dip", (0, 1.0), 2.0),
        ]
    )
    fingers.append(thumb)
    
    # Other fingers: 4 DOF each (abduction + 3 flexion)
    for name in ["index", "middle", "ring", "pinky"]:
        finger = Finger(
            name=name,
            joints=[
                FingerJoint(f"{name}_abduction", (-0.3, 0.3), 2.0),
                FingerJoint(f"{name}_mcp", (0, 1.6), 5.0),
                FingerJoint(f"{name}_pip", (0, 1.8), 3.0),
                FingerJoint(f"{name}_dip", (0, 1.2), 2.0),
            ]
        )
        fingers.append(finger)
    
    return HumanoidHand(
        fingers=fingers,
        palm_sensors=True,
        finger_tip_sensors=True,
        actuation="tendon"
    )


# Check hand capabilities
hand = create_dexterous_hand()
print(f"Hand DOF: {hand.total_dof}")
print(f"Supported grasps: {hand.grasp_types_supported()}")
```

## Comparison of Notable Humanoids

| Robot | Height | Weight | DOF | Key Features |
|-------|--------|--------|-----|--------------|
| **Atlas (Boston Dynamics)** | 1.5m | 89kg | 28+ | Hydraulic, dynamic motion |
| **Optimus (Tesla)** | 1.73m | 57kg | 28 | Electric, manufacturing focus |
| **Figure 01** | 1.68m | 60kg | 40+ | General purpose, AI integration |
| **Digit (Agility)** | 1.75m | 65kg | 16 | Logistics, package handling |
| **ASIMO (Honda)** | 1.3m | 50kg | 57 | Research pioneer, retired |
| **NAO (Aldebaran)** | 0.58m | 5.5kg | 25 | Education, research |

## Design Tradeoffs

```python
class HumanoidDesignOptimizer:
    """Explore design tradeoffs in humanoid construction."""
    
    def analyze_tradeoffs(self):
        """Key design decisions and their implications."""
        
        tradeoffs = {
            "Size": {
                "Larger": ["More payload", "Longer reach", "Higher power needs"],
                "Smaller": ["Safer near humans", "Less capable", "Lower cost"],
            },
            "Actuation": {
                "Electric": ["Quiet", "Clean", "Lower force density"],
                "Hydraulic": ["High force", "Fast", "Noisy, messy"],
                "Pneumatic": ["Compliant", "Light", "Imprecise"],
            },
            "DOF": {
                "High DOF": ["Dexterous", "Complex control", "Expensive"],
                "Low DOF": ["Simple", "Robust", "Limited tasks"],
            },
            "Sensors": {
                "Rich sensing": ["Better perception", "Data processing load"],
                "Minimal sensing": ["Simpler software", "Less adaptive"],
            },
        }
        
        return tradeoffs
```

## Summary

Humanoid robots are marvels of mechanical engineering:

- **Human-like form** enables operation in human environments
- **High DOF** provides dexterity but increases complexity
- **Subsystems** (head, arms, legs, hands) each have specialized requirements
- **Design tradeoffs** between capability, complexity, and cost

<div className="key-takeaways">

#### ✅ Key Takeaways

- Humanoids typically have **30-50+ DOF** for full-body motion
- **Arms** are designed for reach and manipulation (7 DOF typical)
- **Legs** must handle dynamic balance (6 DOF per leg)
- **Hands** are the most complex subsystem (10-25+ DOF)

**Next Chapter**: [Sensors and Actuators →](/docs/humanoid-robotics/sensors-and-actuators)

</div>



