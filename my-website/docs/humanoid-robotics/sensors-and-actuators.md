---
sidebar_position: 2
title: Sensors and Actuators
description: The eyes, ears, and muscles of robotic systems
---

# Sensors and Actuators

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will be able to:
- Classify different sensor types and their applications
- Understand actuator characteristics and selection criteria
- Interface sensors and actuators with control systems
- Choose appropriate sensors/actuators for specific tasks

</div>

## The Perception-Action Interface

Sensors and actuators form the bridge between the robot's software intelligence and the physical world:

```
    PHYSICAL WORLD                    DIGITAL WORLD
         │                                 │
    ┌────▼────┐    Convert to         ┌────▼────┐
    │ Sensors │ ──────────────────▶   │  Data   │
    │         │    Digital Data       │ Process │
    └─────────┘                       │         │
                                      │   AI    │
    ┌─────────┐    Convert to         │  Logic  │
    │Actuators│ ◀──────────────────   │         │
    │         │    Physical Force     └────▲────┘
    └────┬────┘                            │
         │                                 │
         ▼                                 ▼
    PHYSICAL WORLD                    DIGITAL WORLD
```

## Sensor Categories

### Proprioceptive Sensors (Internal State)

These sensors measure the robot's own state:

```python
from dataclasses import dataclass
from enum import Enum
from typing import Tuple

class ProprioceptiveSensors:
    """Sensors that measure the robot's internal state."""
    
    @dataclass
    class JointEncoder:
        """Measures joint angle/position."""
        resolution: int          # Counts per revolution
        absolute: bool           # True = knows position at startup
        max_speed: float         # RPM before missing counts
        
        def position_to_angle(self, counts: int) -> float:
            """Convert encoder counts to radians."""
            import math
            return (counts / self.resolution) * 2 * math.pi
    
    @dataclass
    class TorqueSensor:
        """Measures joint torque/force."""
        range_nm: Tuple[float, float]  # Min, max torque
        resolution: float               # Nm per bit
        bandwidth_hz: float             # Frequency response
    
    @dataclass
    class IMU:
        """Inertial Measurement Unit - orientation and motion."""
        accelerometer_range: float  # ±g
        gyroscope_range: float      # ±deg/s
        magnetometer: bool          # Has compass?
        update_rate: int            # Hz
        
        def estimate_orientation(self, accel, gyro, dt):
            """Fuse accelerometer and gyroscope for orientation."""
            # Simplified complementary filter
            alpha = 0.98  # Trust gyro more for fast motion
            
            # Accelerometer gives pitch/roll (not yaw)
            # Gyroscope gives angular velocity
            # Combine for stable orientation estimate
            pass


# Example usage
encoder = ProprioceptiveSensors.JointEncoder(
    resolution=4096,      # 12-bit encoder
    absolute=True,        # No homing needed
    max_speed=6000        # RPM
)

angle_rad = encoder.position_to_angle(1024)  # 0.25 revolution = π/2 radians
```

### Exteroceptive Sensors (External World)

These sensors perceive the environment:

```python
@dataclass
class Camera:
    """Vision sensor for perceiving the environment."""
    
    class CameraType(Enum):
        RGB = "rgb"
        DEPTH = "depth"
        RGBD = "rgbd"
        STEREO = "stereo"
        EVENT = "event"
    
    camera_type: CameraType
    resolution: Tuple[int, int]
    fov_degrees: float
    frame_rate: int
    
    def project_point(self, point_3d: Tuple[float, float, float], 
                      intrinsics: dict) -> Tuple[int, int]:
        """Project 3D point to 2D pixel coordinates."""
        x, y, z = point_3d
        fx, fy = intrinsics['fx'], intrinsics['fy']
        cx, cy = intrinsics['cx'], intrinsics['cy']
        
        if z <= 0:
            return None  # Behind camera
        
        u = int(fx * x / z + cx)
        v = int(fy * y / z + cy)
        
        return (u, v)


@dataclass
class LiDAR:
    """Light Detection and Ranging sensor."""
    
    scan_type: str           # "2D", "3D"
    points_per_second: int   # Point cloud density
    range_m: Tuple[float, float]  # Min, max range
    angular_resolution: float     # Degrees
    
    def filter_by_range(self, points, min_dist, max_dist):
        """Remove points outside specified range."""
        import numpy as np
        distances = np.linalg.norm(points, axis=1)
        mask = (distances >= min_dist) & (distances <= max_dist)
        return points[mask]


@dataclass
class ForceTorqueSensor:
    """6-axis force/torque sensor for contact sensing."""
    
    force_range: Tuple[float, float, float]   # Fx, Fy, Fz max (N)
    torque_range: Tuple[float, float, float]  # Tx, Ty, Tz max (Nm)
    resolution: float        # N or Nm
    overload_protection: float  # Factor above range
    
    def detect_contact(self, reading, threshold=5.0):
        """Detect if robot is in contact with something."""
        import numpy as np
        force_magnitude = np.linalg.norm(reading[:3])
        return force_magnitude > threshold


@dataclass
class TactileSensor:
    """Touch/pressure sensor for grasping."""
    
    sensor_type: str         # "resistive", "capacitive", "optical"
    spatial_resolution: int  # Taxels (tactile pixels) per cm²
    pressure_range: Tuple[float, float]  # kPa
    
    def detect_grip_slip(self, readings_history):
        """Detect if gripped object is slipping."""
        # Look for rapid changes in pressure pattern
        # that indicate relative motion
        pass
```

### Sensor Comparison Table

| Sensor Type | Range | Resolution | Update Rate | Cost | Use Case |
|-------------|-------|------------|-------------|------|----------|
| **Encoder** | 0-360° | 0.01° | 10 kHz | Low | Joint position |
| **IMU** | ±16g, ±2000°/s | 16-bit | 1 kHz | Low | Orientation |
| **RGB Camera** | 0.3-∞ m | 4K | 60 Hz | Low | Visual recognition |
| **Depth Camera** | 0.2-10 m | mm | 90 Hz | Medium | 3D perception |
| **LiDAR** | 0.1-200 m | cm | 20 Hz | High | Mapping, obstacles |
| **F/T Sensor** | ±500 N | 0.1 N | 1 kHz | High | Contact forces |
| **Tactile** | 0-100 kPa | 1 kPa | 100 Hz | Medium | Grip sensing |

## Actuator Types

### Electric Motors

```python
from enum import Enum
from dataclasses import dataclass

class MotorType(Enum):
    DC_BRUSHED = "dc_brushed"
    DC_BRUSHLESS = "bldc"
    STEPPER = "stepper"
    SERVO = "servo"


@dataclass
class ElectricMotor:
    """Electric motor characteristics."""
    
    motor_type: MotorType
    nominal_voltage: float       # V
    no_load_speed: float        # RPM
    stall_torque: float         # Nm
    continuous_torque: float    # Nm
    peak_torque: float          # Nm
    efficiency: float           # 0-1
    
    def torque_speed_curve(self, speeds):
        """Linear approximation of torque-speed relationship."""
        import numpy as np
        # Torque decreases linearly from stall to no-load speed
        torques = self.stall_torque * (1 - np.array(speeds) / self.no_load_speed)
        return np.maximum(torques, 0)
    
    def power_at_speed(self, speed_rpm):
        """Calculate mechanical power output."""
        import math
        torque = self.stall_torque * (1 - speed_rpm / self.no_load_speed)
        speed_rad_s = speed_rpm * 2 * math.pi / 60
        return torque * speed_rad_s


@dataclass
class ServoActuator:
    """Complete servo actuator with motor, gearbox, encoder."""
    
    motor: ElectricMotor
    gear_ratio: float           # e.g., 100:1
    output_torque: float        # Nm at output
    output_speed: float         # RPM at output
    integrated_encoder: bool
    position_accuracy: float    # degrees
    
    @classmethod
    def from_motor(cls, motor: ElectricMotor, gear_ratio: float):
        """Create servo from motor specifications."""
        return cls(
            motor=motor,
            gear_ratio=gear_ratio,
            output_torque=motor.stall_torque * gear_ratio * 0.9,  # 90% efficiency
            output_speed=motor.no_load_speed / gear_ratio,
            integrated_encoder=True,
            position_accuracy=0.1
        )
```

### Comparison of Actuator Types

```python
class ActuatorComparison:
    """Compare different actuator technologies."""
    
    @staticmethod
    def compare():
        data = {
            "Electric (BLDC)": {
                "force_density": "Medium",
                "speed": "High",
                "efficiency": "85-95%",
                "control": "Excellent",
                "maintenance": "Low",
                "noise": "Low",
                "cost": "Medium",
                "best_for": "Most robotics applications"
            },
            "Hydraulic": {
                "force_density": "Very High",
                "speed": "High", 
                "efficiency": "40-60%",
                "control": "Good",
                "maintenance": "Medium",
                "noise": "High",
                "cost": "High",
                "best_for": "Heavy lifting, dynamic motion"
            },
            "Pneumatic": {
                "force_density": "Low",
                "speed": "Very High",
                "efficiency": "20-30%",
                "control": "Limited",
                "maintenance": "Low",
                "noise": "Medium",
                "cost": "Low",
                "best_for": "Soft robotics, grippers"
            },
            "Series Elastic (SEA)": {
                "force_density": "Medium",
                "speed": "Medium",
                "efficiency": "80-90%",
                "control": "Excellent",
                "maintenance": "Low",
                "noise": "Low",
                "cost": "High",
                "best_for": "Human-safe interaction"
            }
        }
        return data
```

### Series Elastic Actuators (SEA)

Critical for safe human-robot interaction:

```python
@dataclass
class SeriesElasticActuator:
    """
    Actuator with intentional compliance for safety and force control.
    
    Motor ─── Gearbox ─── Spring ─── Output
                           │
                   (Spring deflection = force)
    """
    
    motor: ElectricMotor
    gear_ratio: float
    spring_stiffness: float      # N/m or Nm/rad
    max_deflection: float        # m or rad
    
    def estimate_output_force(self, spring_deflection: float) -> float:
        """Estimate force from spring deflection."""
        return self.spring_stiffness * spring_deflection
    
    def set_force(self, desired_force: float, current_deflection: float):
        """Control motor position to achieve desired output force."""
        current_force = self.estimate_output_force(current_deflection)
        force_error = desired_force - current_force
        
        # Position adjustment needed
        position_adjustment = force_error / self.spring_stiffness
        
        return position_adjustment
    
    @property
    def max_force(self) -> float:
        """Maximum safe force output."""
        return self.spring_stiffness * self.max_deflection
    
    @property 
    def natural_frequency(self) -> float:
        """Natural oscillation frequency of the spring-mass system."""
        import math
        # Simplified: would need reflected inertia
        # Typical SEA: 5-20 Hz
        return 10.0  # Hz placeholder
```

## Sensor Fusion

Combining multiple sensors for robust perception:

```python
import numpy as np
from dataclasses import dataclass
from typing import List

@dataclass
class SensorReading:
    """A single sensor measurement."""
    value: np.ndarray
    uncertainty: np.ndarray  # Covariance
    timestamp: float


class SensorFusion:
    """Combine multiple sensor readings for better estimates."""
    
    @staticmethod
    def weighted_average(readings: List[SensorReading]) -> np.ndarray:
        """Simple weighted average based on uncertainty."""
        
        if not readings:
            return None
        
        weights = []
        values = []
        
        for r in readings:
            # Weight inversely proportional to uncertainty
            w = 1.0 / (np.trace(r.uncertainty) + 1e-6)
            weights.append(w)
            values.append(r.value)
        
        weights = np.array(weights)
        weights /= weights.sum()  # Normalize
        
        fused = sum(w * v for w, v in zip(weights, values))
        return fused
    
    @staticmethod
    def kalman_update(prediction: SensorReading, 
                      measurement: SensorReading) -> SensorReading:
        """Single step of Kalman filter fusion."""
        
        # Kalman gain
        S = prediction.uncertainty + measurement.uncertainty
        K = prediction.uncertainty @ np.linalg.inv(S)
        
        # Update estimate
        innovation = measurement.value - prediction.value
        updated_value = prediction.value + K @ innovation
        
        # Update uncertainty
        I = np.eye(len(prediction.value))
        updated_uncertainty = (I - K) @ prediction.uncertainty
        
        return SensorReading(
            value=updated_value,
            uncertainty=updated_uncertainty,
            timestamp=measurement.timestamp
        )


# Example: Fusing position estimates from different sensors
class PositionEstimator:
    """Fuse encoders, IMU, and vision for position estimate."""
    
    def __init__(self):
        self.encoder_weight = 0.5
        self.imu_weight = 0.3
        self.vision_weight = 0.2
    
    def estimate(self, encoder_pos, imu_pos, vision_pos):
        """Weighted fusion of position estimates."""
        
        # Simple weighted average (real system would use Kalman filter)
        fused_pos = (
            self.encoder_weight * encoder_pos +
            self.imu_weight * imu_pos +
            self.vision_weight * vision_pos
        )
        
        return fused_pos
```

## Practical Interfacing

Example of reading sensors and commanding actuators:

```python
import time
from abc import ABC, abstractmethod

class SensorInterface(ABC):
    """Abstract interface for all sensors."""
    
    @abstractmethod
    def read(self):
        """Read current sensor value."""
        pass
    
    @abstractmethod
    def calibrate(self):
        """Calibrate the sensor."""
        pass


class ActuatorInterface(ABC):
    """Abstract interface for all actuators."""
    
    @abstractmethod
    def command(self, value):
        """Send command to actuator."""
        pass
    
    @abstractmethod
    def get_state(self):
        """Get current actuator state."""
        pass
    
    @abstractmethod
    def emergency_stop(self):
        """Immediately stop the actuator."""
        pass


class JointController:
    """Control a single robot joint with sensor and actuator."""
    
    def __init__(self, encoder: SensorInterface, motor: ActuatorInterface):
        self.encoder = encoder
        self.motor = motor
        self.target_position = 0.0
        self.kp = 10.0  # Proportional gain
        
    def update(self, dt: float):
        """Run one control cycle."""
        
        # Read current position
        current_pos = self.encoder.read()
        
        # Calculate error
        error = self.target_position - current_pos
        
        # Simple proportional control
        command = self.kp * error
        
        # Send to motor
        self.motor.command(command)
        
        return current_pos, error
    
    def move_to(self, target: float, timeout: float = 5.0):
        """Move to target position."""
        
        self.target_position = target
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            pos, error = self.update(0.01)
            
            if abs(error) < 0.01:  # Within tolerance
                return True
            
            time.sleep(0.01)
        
        return False  # Timeout
```

## Summary

Sensors and actuators are the physical interface of intelligent robots:

- **Sensors** convert physical phenomena to digital data
- **Actuators** convert digital commands to physical motion
- **Selection** depends on application requirements
- **Fusion** combines multiple sensors for robustness

<div className="key-takeaways">

#### ✅ Key Takeaways

- **Proprioceptive sensors** measure internal state (encoders, IMU, torque)
- **Exteroceptive sensors** perceive the environment (cameras, LiDAR, touch)
- **Electric motors** dominate modern robotics for their control precision
- **Series Elastic Actuators** enable safe human-robot interaction

**Next Chapter**: [Kinematics Fundamentals →](/docs/humanoid-robotics/kinematics-fundamentals)

</div>



