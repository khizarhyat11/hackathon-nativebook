---
sidebar_position: 2
title: Agent Architectures
description: Designing robust agent architectures for robotic systems
---

# Agent Architectures

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will be able to:
- Compare different agent architecture paradigms
- Implement layered and behavior-based architectures
- Design cognitive architectures for complex tasks
- Choose appropriate architectures for specific applications

</div>

## Architecture Overview

Agent architecture defines how perception, reasoning, and action components are organized:

```
┌──────────────────────────────────────────────────────────────────┐
│                     AGENT ARCHITECTURE SPECTRUM                  │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  REACTIVE ◄──────────────────────────────────────► DELIBERATIVE │
│                                                                  │
│  • Fast response          HYBRID              • Deep reasoning   │
│  • No planning        (combines both)         • Slow response    │
│  • Simple behaviors                           • Complex planning │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

## Subsumption Architecture

Pioneered by Rodney Brooks, behaviors are layered with priorities:

```python
from abc import ABC, abstractmethod
from typing import Optional, List
from dataclasses import dataclass

@dataclass
class Behavior:
    """A single behavior module."""
    name: str
    priority: int  # Higher = more important
    
    @abstractmethod
    def is_active(self, percepts: dict) -> bool:
        """Check if this behavior should activate."""
        pass
    
    @abstractmethod
    def compute_action(self, percepts: dict) -> dict:
        """Compute action if active."""
        pass


class SubsumptionArchitecture:
    """
    Layered behavior-based architecture.
    
    Higher priority behaviors subsume (override) lower priority ones.
    """
    
    def __init__(self):
        self.behaviors: List[Behavior] = []
    
    def add_behavior(self, behavior: Behavior):
        """Add behavior and maintain priority order."""
        self.behaviors.append(behavior)
        self.behaviors.sort(key=lambda b: -b.priority)
    
    def compute_action(self, percepts: dict) -> Optional[dict]:
        """
        Compute action by checking behaviors in priority order.
        First active behavior wins.
        """
        for behavior in self.behaviors:
            if behavior.is_active(percepts):
                action = behavior.compute_action(percepts)
                return {
                    "action": action,
                    "source": behavior.name
                }
        
        return None


# Example behaviors for a mobile robot
class AvoidCollision(Behavior):
    """Highest priority: avoid hitting things."""
    
    def __init__(self):
        super().__init__("avoid_collision", priority=10)
    
    def is_active(self, percepts: dict) -> bool:
        return percepts.get("min_distance", 100) < 0.3
    
    def compute_action(self, percepts: dict) -> dict:
        return {"type": "stop", "turn": 45}


class FollowWall(Behavior):
    """Medium priority: follow walls for navigation."""
    
    def __init__(self):
        super().__init__("follow_wall", priority=5)
    
    def is_active(self, percepts: dict) -> bool:
        wall_distance = percepts.get("right_distance", 100)
        return 0.3 < wall_distance < 1.0
    
    def compute_action(self, percepts: dict) -> dict:
        error = percepts["right_distance"] - 0.5
        return {"type": "move", "speed": 0.3, "turn": -error * 30}


class Wander(Behavior):
    """Lowest priority: default wandering."""
    
    def __init__(self):
        super().__init__("wander", priority=1)
    
    def is_active(self, percepts: dict) -> bool:
        return True  # Always ready
    
    def compute_action(self, percepts: dict) -> dict:
        import random
        return {"type": "move", "speed": 0.5, "turn": random.uniform(-10, 10)}


# Usage
robot = SubsumptionArchitecture()
robot.add_behavior(AvoidCollision())
robot.add_behavior(FollowWall())
robot.add_behavior(Wander())

percepts = {"min_distance": 0.2, "right_distance": 0.8}
result = robot.compute_action(percepts)
print(f"Action from {result['source']}: {result['action']}")
```

## Three-Layer Architecture

Combines reactive, executive, and deliberative layers:

```python
from enum import Enum
from threading import Thread, Lock
import time

class LayerType(Enum):
    REACTIVE = "reactive"      # Fast, direct response
    EXECUTIVE = "executive"    # Sequencing, monitoring
    DELIBERATIVE = "deliberative"  # Planning, reasoning


class ThreeLayerArchitecture:
    """
    Classic three-layer robot architecture.
    
    Deliberative Layer: Plans and reasons (slow, 1-10 Hz)
    Executive Layer: Sequences behaviors (medium, 10-50 Hz)
    Reactive Layer: Direct control (fast, 100+ Hz)
    """
    
    def __init__(self):
        self.reactive_layer = ReactiveLayer()
        self.executive_layer = ExecutiveLayer()
        self.deliberative_layer = DeliberativeLayer()
        
        self._running = False
        self._lock = Lock()
    
    def start(self):
        """Start all layers in separate threads."""
        self._running = True
        
        # Each layer runs at different frequency
        Thread(target=self._run_reactive, daemon=True).start()
        Thread(target=self._run_executive, daemon=True).start()
        Thread(target=self._run_deliberative, daemon=True).start()
    
    def _run_reactive(self):
        """Reactive loop at 100 Hz."""
        while self._running:
            with self._lock:
                self.reactive_layer.update()
            time.sleep(0.01)
    
    def _run_executive(self):
        """Executive loop at 20 Hz."""
        while self._running:
            with self._lock:
                self.executive_layer.update(self.reactive_layer)
            time.sleep(0.05)
    
    def _run_deliberative(self):
        """Deliberative loop at 2 Hz."""
        while self._running:
            with self._lock:
                self.deliberative_layer.update(self.executive_layer)
            time.sleep(0.5)


class ReactiveLayer:
    """Fast, direct sensor-to-actuator mapping."""
    
    def __init__(self):
        self.current_command = None
        self.safety_active = False
    
    def update(self):
        """Process sensor data and generate motor commands."""
        # Direct safety behaviors
        # E.g., collision avoidance, joint limits
        pass
    
    def set_command(self, command):
        """Receive command from executive layer."""
        if not self.safety_active:
            self.current_command = command


class ExecutiveLayer:
    """Behavior sequencing and monitoring."""
    
    def __init__(self):
        self.current_behavior = None
        self.behavior_queue = []
    
    def update(self, reactive: ReactiveLayer):
        """Execute current behavior, monitor progress."""
        
        if self.current_behavior is None and self.behavior_queue:
            self.current_behavior = self.behavior_queue.pop(0)
        
        if self.current_behavior:
            command = self.current_behavior.step()
            reactive.set_command(command)
            
            if self.current_behavior.is_complete():
                self.current_behavior = None
    
    def set_plan(self, behaviors: list):
        """Receive plan from deliberative layer."""
        self.behavior_queue = behaviors


class DeliberativeLayer:
    """High-level planning and reasoning."""
    
    def __init__(self):
        self.current_goal = None
        self.world_model = {}
    
    def update(self, executive: ExecutiveLayer):
        """Plan to achieve goals."""
        
        if self.current_goal:
            plan = self._create_plan(self.current_goal)
            executive.set_plan(plan)
    
    def _create_plan(self, goal) -> list:
        """Generate plan for goal."""
        # Would use actual planning algorithm
        return []
    
    def set_goal(self, goal):
        """Accept new goal."""
        self.current_goal = goal
```

## BDI Architecture (Belief-Desire-Intention)

Models agent reasoning with beliefs, desires, and intentions:

```python
from dataclasses import dataclass, field
from typing import Set, Dict, Any

@dataclass
class Belief:
    """Something the agent believes about the world."""
    predicate: str
    arguments: tuple
    confidence: float = 1.0
    
    def __hash__(self):
        return hash((self.predicate, self.arguments))


@dataclass
class Desire:
    """Something the agent wants to achieve."""
    description: str
    priority: int
    achievable: bool = True


@dataclass
class Intention:
    """A commitment to achieve a goal."""
    desire: Desire
    plan: list
    current_step: int = 0


class BDIAgent:
    """
    Belief-Desire-Intention agent architecture.
    
    - Beliefs: What the agent thinks is true
    - Desires: What the agent wants
    - Intentions: What the agent is committed to doing
    """
    
    def __init__(self):
        self.beliefs: Set[Belief] = set()
        self.desires: List[Desire] = []
        self.intentions: List[Intention] = []
        self.plan_library: Dict[str, callable] = {}
    
    def add_belief(self, belief: Belief):
        """Add or update a belief."""
        # Remove conflicting beliefs
        self.beliefs = {b for b in self.beliefs 
                       if b.predicate != belief.predicate or 
                       b.arguments != belief.arguments}
        self.beliefs.add(belief)
    
    def add_desire(self, desire: Desire):
        """Add a new desire."""
        self.desires.append(desire)
        self.desires.sort(key=lambda d: -d.priority)
    
    def deliberate(self):
        """
        Deliberation: Select which desires to pursue.
        Creates intentions from achievable, compatible desires.
        """
        for desire in self.desires:
            if desire.achievable and self._compatible_with_intentions(desire):
                plan = self._find_plan(desire)
                if plan:
                    intention = Intention(desire, plan)
                    self.intentions.append(intention)
    
    def means_ends_reasoning(self):
        """
        Means-ends reasoning: How to achieve intentions.
        Updates plans based on current beliefs.
        """
        for intention in self.intentions:
            if not self._plan_still_valid(intention):
                new_plan = self._find_plan(intention.desire)
                if new_plan:
                    intention.plan = new_plan
                    intention.current_step = 0
                else:
                    intention.desire.achievable = False
    
    def execute_step(self):
        """Execute one step of the current intention."""
        if not self.intentions:
            return None
        
        intention = self.intentions[0]
        
        if intention.current_step >= len(intention.plan):
            # Intention complete
            self.intentions.pop(0)
            return None
        
        action = intention.plan[intention.current_step]
        intention.current_step += 1
        
        return action
    
    def perceive(self, observations: dict):
        """Update beliefs based on observations."""
        for key, value in observations.items():
            belief = Belief(key, (value,))
            self.add_belief(belief)
    
    def _compatible_with_intentions(self, desire: Desire) -> bool:
        """Check if desire is compatible with current intentions."""
        # Simplified: check for conflicts
        return True
    
    def _find_plan(self, desire: Desire) -> list:
        """Find a plan to achieve desire."""
        if desire.description in self.plan_library:
            return self.plan_library[desire.description](self.beliefs)
        return []
    
    def _plan_still_valid(self, intention: Intention) -> bool:
        """Check if plan is still valid given current beliefs."""
        return True


# Example usage
agent = BDIAgent()

# Add initial beliefs
agent.add_belief(Belief("at", ("robot", "room_A")))
agent.add_belief(Belief("battery_level", (80,)))

# Add desires
agent.add_desire(Desire("deliver_package_to_room_B", priority=5))

# Register plan for delivery
def delivery_plan(beliefs):
    return [
        {"action": "pick_up", "object": "package"},
        {"action": "navigate", "destination": "room_B"},
        {"action": "put_down", "object": "package"},
    ]

agent.plan_library["deliver_package_to_room_B"] = delivery_plan

# Run BDI cycle
agent.deliberate()
agent.means_ends_reasoning()
action = agent.execute_step()
print(f"Executing: {action}")
```

## Hybrid Architectures for Modern Robots

Combining classical AI with modern deep learning:

```python
class ModernHybridArchitecture:
    """
    Modern hybrid architecture combining:
    - Deep learning for perception
    - Classical planning for reasoning
    - Learned policies for control
    """
    
    def __init__(self):
        # Perception: Neural networks
        self.vision_model = None  # e.g., YOLO, SAM
        self.language_model = None  # e.g., GPT for understanding
        
        # World model: Symbolic + neural
        self.symbolic_state = {}
        self.neural_state_encoder = None
        
        # Planning: Classical + learned
        self.task_planner = None  # PDDL planner
        self.motion_planner = None  # RRT + learned heuristics
        
        # Control: Learned policies
        self.manipulation_policy = None
        self.navigation_policy = None
    
    def process_scene(self, rgb_image, depth_image):
        """Neural perception pipeline."""
        
        # Object detection
        objects = self.vision_model.detect(rgb_image)
        
        # Scene understanding
        scene_description = self.language_model.describe(rgb_image)
        
        # Convert to symbolic representation
        for obj in objects:
            self.symbolic_state[obj.id] = {
                "class": obj.class_name,
                "position": obj.position,
                "confidence": obj.confidence
            }
        
        return objects, scene_description
    
    def plan_task(self, natural_language_instruction: str):
        """
        High-level task planning from language.
        Uses LLM for task decomposition.
        """
        
        # LLM converts instruction to plan
        prompt = f"""
        Given the scene: {self.symbolic_state}
        Instruction: {natural_language_instruction}
        Generate a sequence of primitive actions.
        """
        
        # plan = self.language_model.generate(prompt)
        plan = [
            {"action": "grasp", "object": "cup"},
            {"action": "move", "position": [0.5, 0.3, 0.1]},
            {"action": "release"}
        ]
        
        return plan
    
    def execute_action(self, action: dict):
        """Execute action using learned policy."""
        
        if action["action"] == "grasp":
            # Use manipulation policy
            trajectory = self.manipulation_policy.grasp(action["object"])
            return trajectory
        
        elif action["action"] == "move":
            # Use motion planner + navigation policy
            path = self.motion_planner.plan(action["position"])
            return path
        
        return None
```

## Summary

Agent architectures provide the organizational structure for intelligent robots:

- **Subsumption** is simple and reactive
- **Three-layer** balances reaction and deliberation  
- **BDI** models rational agent reasoning
- **Hybrid** architectures combine learning with classical methods

<div className="key-takeaways">

#### ✅ Key Takeaways

- **Reactive architectures** are fast but limited
- **Deliberative architectures** can reason but are slow
- **Hybrid architectures** provide the best of both worlds
- Modern systems combine **neural perception with symbolic reasoning**

**Next Chapter**: [Perception Systems →](/docs/ai-agent-integration/perception-systems)

</div>



