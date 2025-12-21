---
sidebar_position: 4
title: Motion Planning
description: Planning collision-free paths for robot movement
---

# Motion Planning

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will be able to:
- Understand the motion planning problem and its variants
- Implement basic path planning algorithms
- Apply trajectory smoothing and optimization
- Handle dynamic obstacles and constraints

</div>

## The Motion Planning Problem

Motion planning answers: "How do I get from here to there without hitting anything?"

```python
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np

@dataclass
class PlanningProblem:
    """Definition of a motion planning problem."""
    
    start: np.ndarray          # Start configuration
    goal: np.ndarray           # Goal configuration  
    obstacles: List            # Obstacles in the environment
    joint_limits: List[Tuple]  # Min/max for each joint
    velocity_limits: List      # Max velocity per joint
    
    def is_goal(self, config: np.ndarray, tolerance: float = 0.1) -> bool:
        """Check if configuration is close to goal."""
        return np.linalg.norm(config - self.goal) < tolerance


@dataclass
class Path:
    """A sequence of configurations forming a path."""
    
    waypoints: List[np.ndarray]
    
    @property
    def length(self) -> float:
        """Total path length in configuration space."""
        total = 0.0
        for i in range(len(self.waypoints) - 1):
            total += np.linalg.norm(self.waypoints[i+1] - self.waypoints[i])
        return total
    
    def interpolate(self, num_points: int) -> 'Path':
        """Interpolate to create smoother path."""
        if len(self.waypoints) < 2:
            return self
        
        new_waypoints = []
        for i in range(len(self.waypoints) - 1):
            start = self.waypoints[i]
            end = self.waypoints[i + 1]
            
            for t in np.linspace(0, 1, num_points // (len(self.waypoints) - 1)):
                new_waypoints.append(start + t * (end - start))
        
        new_waypoints.append(self.waypoints[-1])
        return Path(new_waypoints)
```

## Configuration Space

The configuration space (C-space) represents all possible robot configurations:

```python
class ConfigurationSpace:
    """Robot configuration space representation."""
    
    def __init__(self, robot, obstacles):
        self.robot = robot
        self.obstacles = obstacles
    
    def is_collision_free(self, config: np.ndarray) -> bool:
        """Check if configuration collides with obstacles."""
        
        # Get robot geometry at this configuration
        robot_points = self.robot.get_geometry_at(config)
        
        for obstacle in self.obstacles:
            if self._check_collision(robot_points, obstacle):
                return False
        
        return True
    
    def is_edge_valid(self, config1: np.ndarray, config2: np.ndarray,
                      num_checks: int = 10) -> bool:
        """Check if straight-line path between configs is collision-free."""
        
        for t in np.linspace(0, 1, num_checks):
            config = config1 + t * (config2 - config1)
            if not self.is_collision_free(config):
                return False
        
        return True
    
    def _check_collision(self, robot_points, obstacle) -> bool:
        """Check collision between robot and single obstacle."""
        # Simplified: sphere-sphere collision
        for point in robot_points:
            distance = np.linalg.norm(point - obstacle.center)
            if distance < obstacle.radius:
                return True
        return False
```

## Rapidly-exploring Random Trees (RRT)

A popular sampling-based planner:

```python
import random

class RRT:
    """Rapidly-exploring Random Tree planner."""
    
    def __init__(self, c_space: ConfigurationSpace, 
                 step_size: float = 0.1,
                 goal_bias: float = 0.1):
        self.c_space = c_space
        self.step_size = step_size
        self.goal_bias = goal_bias
    
    def plan(self, start: np.ndarray, goal: np.ndarray, 
             max_iterations: int = 1000) -> Optional[Path]:
        """
        Plan a path from start to goal.
        
        Returns Path if successful, None if planning fails.
        """
        
        # Tree structure: list of (config, parent_index)
        tree = [(start, None)]
        
        for i in range(max_iterations):
            # Sample random configuration (with goal bias)
            if random.random() < self.goal_bias:
                q_rand = goal
            else:
                q_rand = self._random_config()
            
            # Find nearest node in tree
            nearest_idx = self._find_nearest(tree, q_rand)
            q_nearest = tree[nearest_idx][0]
            
            # Extend toward random config
            q_new = self._extend(q_nearest, q_rand)
            
            # Check if extension is valid
            if self.c_space.is_edge_valid(q_nearest, q_new):
                tree.append((q_new, nearest_idx))
                
                # Check if we reached the goal
                if np.linalg.norm(q_new - goal) < self.step_size:
                    # Connect to goal
                    if self.c_space.is_edge_valid(q_new, goal):
                        tree.append((goal, len(tree) - 1))
                        return self._extract_path(tree)
        
        return None  # Planning failed
    
    def _random_config(self) -> np.ndarray:
        """Sample random configuration."""
        # Assuming joint limits stored in c_space
        config = []
        for low, high in self.c_space.robot.joint_limits:
            config.append(random.uniform(low, high))
        return np.array(config)
    
    def _find_nearest(self, tree, q) -> int:
        """Find index of nearest node in tree."""
        distances = [np.linalg.norm(node[0] - q) for node in tree]
        return np.argmin(distances)
    
    def _extend(self, q_near: np.ndarray, q_rand: np.ndarray) -> np.ndarray:
        """Extend from q_near toward q_rand by step_size."""
        direction = q_rand - q_near
        distance = np.linalg.norm(direction)
        
        if distance < self.step_size:
            return q_rand
        
        return q_near + (direction / distance) * self.step_size
    
    def _extract_path(self, tree) -> Path:
        """Extract path from tree by backtracking from goal."""
        path = []
        idx = len(tree) - 1
        
        while idx is not None:
            path.append(tree[idx][0])
            idx = tree[idx][1]
        
        path.reverse()
        return Path(path)
```

## Path Smoothing

Raw RRT paths are often jerky. Smoothing creates better trajectories:

```python
class PathSmoother:
    """Smooth paths for better execution."""
    
    def __init__(self, c_space: ConfigurationSpace):
        self.c_space = c_space
    
    def shortcut(self, path: Path, iterations: int = 100) -> Path:
        """
        Shortcut path by randomly trying to connect non-adjacent waypoints.
        """
        waypoints = path.waypoints.copy()
        
        for _ in range(iterations):
            if len(waypoints) < 3:
                break
            
            # Pick two random non-adjacent points
            i = random.randint(0, len(waypoints) - 3)
            j = random.randint(i + 2, len(waypoints) - 1)
            
            # Try to connect directly
            if self.c_space.is_edge_valid(waypoints[i], waypoints[j]):
                # Remove intermediate points
                waypoints = waypoints[:i+1] + waypoints[j:]
        
        return Path(waypoints)
    
    def bezier_smooth(self, path: Path) -> Path:
        """Smooth path using Bezier curves."""
        if len(path.waypoints) < 3:
            return path
        
        smooth_waypoints = [path.waypoints[0]]
        
        for i in range(len(path.waypoints) - 2):
            p0 = path.waypoints[i]
            p1 = path.waypoints[i + 1]
            p2 = path.waypoints[i + 2]
            
            # Quadratic Bezier
            for t in np.linspace(0, 1, 10):
                point = (1-t)**2 * p0 + 2*(1-t)*t * p1 + t**2 * p2
                smooth_waypoints.append(point)
        
        smooth_waypoints.append(path.waypoints[-1])
        return Path(smooth_waypoints)
```

## Trajectory Generation

Converting a path to a time-parameterized trajectory:

```python
@dataclass
class TrajectoryPoint:
    """Single point in a trajectory."""
    time: float
    position: np.ndarray
    velocity: np.ndarray
    acceleration: np.ndarray


class TrajectoryGenerator:
    """Generate smooth trajectories from paths."""
    
    def __init__(self, max_velocities: np.ndarray, 
                 max_accelerations: np.ndarray):
        self.max_vel = max_velocities
        self.max_acc = max_accelerations
    
    def time_parameterize(self, path: Path, 
                          total_time: float = None) -> List[TrajectoryPoint]:
        """
        Convert path to time-parameterized trajectory.
        """
        waypoints = path.waypoints
        n_points = len(waypoints)
        
        if total_time is None:
            # Estimate based on path length and velocity limits
            total_time = path.length / self.max_vel.min()
        
        trajectory = []
        times = np.linspace(0, total_time, n_points)
        
        for i, (t, pos) in enumerate(zip(times, waypoints)):
            # Compute velocity (finite difference)
            if i == 0:
                vel = (waypoints[1] - waypoints[0]) / (times[1] - times[0])
            elif i == n_points - 1:
                vel = np.zeros_like(pos)  # Stop at end
            else:
                dt = times[i+1] - times[i-1]
                vel = (waypoints[i+1] - waypoints[i-1]) / dt
            
            # Clip to velocity limits
            vel = np.clip(vel, -self.max_vel, self.max_vel)
            
            # Compute acceleration (finite difference of velocity)
            acc = np.zeros_like(pos)  # Simplified
            
            trajectory.append(TrajectoryPoint(t, pos, vel, acc))
        
        return trajectory
    
    def trapezoidal_velocity(self, start: np.ndarray, 
                              end: np.ndarray,
                              via_points: int = 100) -> List[TrajectoryPoint]:
        """
        Generate trajectory with trapezoidal velocity profile.
        
        Accelerate -> Cruise -> Decelerate
        """
        displacement = end - start
        distance = np.linalg.norm(displacement)
        direction = displacement / distance if distance > 0 else np.zeros_like(start)
        
        # Use minimum of limits across all joints
        v_max = np.min(self.max_vel)
        a_max = np.min(self.max_acc)
        
        # Time to accelerate to max velocity
        t_accel = v_max / a_max
        
        # Distance during acceleration/deceleration
        d_accel = 0.5 * a_max * t_accel**2
        
        if 2 * d_accel > distance:
            # Triangular profile (can't reach max velocity)
            t_accel = np.sqrt(distance / a_max)
            t_cruise = 0
            t_total = 2 * t_accel
        else:
            # Trapezoidal profile
            d_cruise = distance - 2 * d_accel
            t_cruise = d_cruise / v_max
            t_total = 2 * t_accel + t_cruise
        
        # Generate trajectory points
        trajectory = []
        times = np.linspace(0, t_total, via_points)
        
        for t in times:
            if t <= t_accel:
                # Acceleration phase
                d = 0.5 * a_max * t**2
                v = a_max * t
                a = a_max
            elif t <= t_accel + t_cruise:
                # Cruise phase
                d = d_accel + v_max * (t - t_accel)
                v = v_max
                a = 0
            else:
                # Deceleration phase
                t_decel = t - t_accel - t_cruise
                d = d_accel + v_max * t_cruise + v_max * t_decel - 0.5 * a_max * t_decel**2
                v = v_max - a_max * t_decel
                a = -a_max
            
            pos = start + d * direction
            vel = v * direction
            acc = a * direction
            
            trajectory.append(TrajectoryPoint(t, pos, vel, acc))
        
        return trajectory
```

## Summary

Motion planning enables robots to navigate complex environments:

- **Configuration space** transforms obstacle avoidance to geometry
- **RRT** and similar algorithms find paths in high-dimensional spaces
- **Path smoothing** improves execution quality
- **Trajectory generation** adds time and respects dynamic limits

<div className="key-takeaways">

#### ✅ Key Takeaways

- Motion planning finds paths from start to goal avoiding obstacles
- **Sampling-based planners** (RRT) scale well to high dimensions
- **Paths need smoothing** before execution
- **Trajectories** include time, velocity, and acceleration

**Next Part**: [Part 3: AI Agent Integration →](/docs/category/ai-robot-brain)

</div>



