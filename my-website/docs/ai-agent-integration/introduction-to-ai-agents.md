---
sidebar_position: 1
title: Introduction to AI Agents
description: Understanding intelligent agents for robotic systems
---

# Introduction to AI Agents

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will be able to:
- Define AI agents and their core properties
- Understand agent architectures for robotics
- Distinguish between different types of agents
- Design simple autonomous agents

</div>

## What is an AI Agent?

An **AI Agent** is a system that perceives its environment and takes actions to achieve goals. In the context of Physical AI, agents are the "brains" that make robots intelligent.

```python
from abc import ABC, abstractmethod
from dataclasses import dataclass
from typing import Any, List

@dataclass
class Percept:
    """Information perceived from the environment."""
    sensor_data: dict
    timestamp: float


@dataclass
class Action:
    """Action to be executed in the environment."""
    action_type: str
    parameters: dict


class Agent(ABC):
    """Abstract base class for all agents."""
    
    @abstractmethod
    def perceive(self, environment) -> Percept:
        """Gather information from the environment."""
        pass
    
    @abstractmethod
    def decide(self, percept: Percept) -> Action:
        """Decide what action to take."""
        pass
    
    @abstractmethod
    def act(self, action: Action, environment) -> Any:
        """Execute the action in the environment."""
        pass
    
    def run_step(self, environment) -> Any:
        """Execute one agent cycle."""
        percept = self.perceive(environment)
        action = self.decide(percept)
        result = self.act(action, environment)
        return result
```

## Agent Properties

Intelligent agents exhibit key properties that distinguish them from simple programs:

```python
class AgentProperties:
    """Key properties of intelligent agents."""
    
    AUTONOMY = """
    Agents operate without direct human intervention.
    They control their own actions and internal state.
    """
    
    REACTIVITY = """
    Agents perceive their environment and respond in a timely fashion.
    They react to changes and unexpected events.
    """
    
    PROACTIVITY = """
    Agents don't just react - they take initiative.
    They exhibit goal-directed behavior to achieve objectives.
    """
    
    SOCIAL_ABILITY = """
    Agents can interact with other agents (and humans).
    They communicate, negotiate, and coordinate.
    """
    
    LEARNING = """
    Agents improve their behavior over time.
    They adapt based on experience.
    """
```

## Types of Agents

### Simple Reflex Agents

React directly to current percept without memory:

```python
class SimpleReflexAgent(Agent):
    """
    Agent that maps percepts directly to actions.
    No memory, no reasoning about future.
    """
    
    def __init__(self, rules: dict):
        """
        rules: dictionary mapping conditions to actions
        """
        self.rules = rules
    
    def perceive(self, environment) -> Percept:
        return Percept(
            sensor_data=environment.get_sensor_readings(),
            timestamp=environment.current_time
        )
    
    def decide(self, percept: Percept) -> Action:
        """Match percept to rules and return action."""
        
        for condition, action in self.rules.items():
            if self._matches(percept, condition):
                return action
        
        return Action("idle", {})
    
    def act(self, action: Action, environment):
        return environment.execute(action)
    
    def _matches(self, percept: Percept, condition) -> bool:
        """Check if percept matches condition."""
        # Simplified pattern matching
        return condition(percept.sensor_data)


# Example: Simple obstacle avoidance
rules = {
    lambda s: s.get('front_distance', 100) < 0.5: Action("turn_left", {"angle": 45}),
    lambda s: s.get('left_distance', 100) < 0.3: Action("turn_right", {"angle": 30}),
    lambda s: True: Action("move_forward", {"speed": 0.5}),
}

agent = SimpleReflexAgent(rules)
```

### Model-Based Agents

Maintain internal state to track the world:

```python
class ModelBasedAgent(Agent):
    """
    Agent that maintains an internal model of the world.
    Uses memory to handle partial observability.
    """
    
    def __init__(self):
        self.world_model = {}  # Internal state
        self.action_history = []
    
    def perceive(self, environment) -> Percept:
        sensor_data = environment.get_sensor_readings()
        
        # Update world model with new observations
        self.update_model(sensor_data)
        
        return Percept(sensor_data, environment.current_time)
    
    def update_model(self, sensor_data: dict):
        """Update internal world model based on observations."""
        
        for key, value in sensor_data.items():
            if key not in self.world_model:
                self.world_model[key] = []
            
            # Keep history
            self.world_model[key].append({
                'value': value,
                'timestamp': len(self.world_model[key])
            })
            
            # Limit history size
            if len(self.world_model[key]) > 100:
                self.world_model[key].pop(0)
    
    def predict_next_state(self, action: Action) -> dict:
        """Predict world state after taking action."""
        # Use internal model to simulate effect
        predicted = self.world_model.copy()
        # Apply action effects...
        return predicted
    
    def decide(self, percept: Percept) -> Action:
        """Decide based on internal model, not just current percept."""
        
        # Can now reason about unobserved aspects
        # and predict effects of actions
        
        if self._is_moving_toward_obstacle():
            return Action("stop", {})
        
        return Action("continue", {})
    
    def _is_moving_toward_obstacle(self) -> bool:
        """Use history to detect convergence with obstacle."""
        if 'front_distance' not in self.world_model:
            return False
        
        history = self.world_model['front_distance']
        if len(history) < 3:
            return False
        
        # Check if distance is decreasing
        recent = [h['value'] for h in history[-3:]]
        return recent[0] > recent[1] > recent[2] and recent[2] < 1.0
    
    def act(self, action: Action, environment):
        self.action_history.append(action)
        return environment.execute(action)
```

### Goal-Based Agents

Plan and act to achieve specific goals:

```python
from typing import Optional

@dataclass
class Goal:
    """Representation of an agent goal."""
    description: str
    success_condition: callable
    priority: int = 1


class GoalBasedAgent(Agent):
    """
    Agent that plans actions to achieve goals.
    Uses search or planning algorithms.
    """
    
    def __init__(self, goals: List[Goal]):
        self.goals = sorted(goals, key=lambda g: -g.priority)
        self.current_goal: Optional[Goal] = None
        self.current_plan: List[Action] = []
        self.world_model = {}
    
    def perceive(self, environment) -> Percept:
        sensor_data = environment.get_sensor_readings()
        self.world_model.update(sensor_data)
        return Percept(sensor_data, environment.current_time)
    
    def decide(self, percept: Percept) -> Action:
        """Select action based on goals and planning."""
        
        # Check if current goal is achieved
        if self.current_goal and self.current_goal.success_condition(self.world_model):
            self.goals.remove(self.current_goal)
            self.current_goal = None
            self.current_plan = []
        
        # Select next goal if needed
        if self.current_goal is None and self.goals:
            self.current_goal = self.goals[0]
            self.current_plan = self.plan_for_goal(self.current_goal)
        
        # Execute next action in plan
        if self.current_plan:
            return self.current_plan.pop(0)
        
        return Action("idle", {})
    
    def plan_for_goal(self, goal: Goal) -> List[Action]:
        """Generate plan to achieve goal."""
        
        # Simple planning - would use actual planner
        if "reach_position" in goal.description:
            target = goal.description.split()[-1]
            return [
                Action("compute_path", {"target": target}),
                Action("follow_path", {}),
            ]
        
        return []
    
    def act(self, action: Action, environment):
        return environment.execute(action)
```

### Learning Agents

Improve performance through experience:

```python
class LearningAgent(Agent):
    """
    Agent that learns from experience.
    Improves its decision-making over time.
    """
    
    def __init__(self):
        self.knowledge_base = {}
        self.performance_history = []
        self.learning_rate = 0.1
    
    def perceive(self, environment) -> Percept:
        return Percept(
            environment.get_sensor_readings(),
            environment.current_time
        )
    
    def decide(self, percept: Percept) -> Action:
        """Choose action based on learned knowledge."""
        
        state = self._extract_state(percept)
        
        # Exploit: use learned knowledge
        if state in self.knowledge_base:
            action_values = self.knowledge_base[state]
            best_action = max(action_values, key=action_values.get)
            return Action(best_action, {})
        
        # Explore: try new actions
        return self._explore()
    
    def learn(self, state, action, reward, next_state):
        """Update knowledge based on experience."""
        
        if state not in self.knowledge_base:
            self.knowledge_base[state] = {}
        
        if action not in self.knowledge_base[state]:
            self.knowledge_base[state][action] = 0.0
        
        # Q-learning update
        current_value = self.knowledge_base[state][action]
        
        if next_state in self.knowledge_base:
            max_next = max(self.knowledge_base[next_state].values())
        else:
            max_next = 0.0
        
        new_value = current_value + self.learning_rate * (
            reward + 0.9 * max_next - current_value
        )
        
        self.knowledge_base[state][action] = new_value
    
    def _extract_state(self, percept: Percept) -> str:
        """Convert percept to discrete state."""
        # Simplified discretization
        data = percept.sensor_data
        return str(data)
    
    def _explore(self) -> Action:
        """Random exploration."""
        import random
        actions = ["forward", "left", "right", "stop"]
        return Action(random.choice(actions), {})
    
    def act(self, action: Action, environment):
        return environment.execute(action)
```

## Agent-Environment Interaction

The complete interaction loop:

```python
class Environment:
    """Simulation environment for agents."""
    
    def __init__(self):
        self.state = {}
        self.current_time = 0.0
        self.agents = []
    
    def add_agent(self, agent: Agent):
        self.agents.append(agent)
    
    def get_sensor_readings(self) -> dict:
        """Provide sensor data to agents."""
        return {
            "position": self.state.get("agent_position", [0, 0, 0]),
            "front_distance": self.state.get("front_obstacle_distance", 10.0),
            "battery": self.state.get("battery_level", 100),
        }
    
    def execute(self, action: Action):
        """Execute action and return result."""
        
        if action.action_type == "move_forward":
            speed = action.parameters.get("speed", 0.1)
            pos = self.state.get("agent_position", [0, 0, 0])
            pos[0] += speed
            self.state["agent_position"] = pos
            return {"success": True, "new_position": pos}
        
        elif action.action_type == "turn_left":
            angle = action.parameters.get("angle", 15)
            # Update orientation...
            return {"success": True}
        
        return {"success": False, "error": "Unknown action"}
    
    def step(self, dt: float = 0.1):
        """Advance simulation by dt seconds."""
        self.current_time += dt
        
        for agent in self.agents:
            agent.run_step(self)


# Example usage
env = Environment()
agent = GoalBasedAgent([
    Goal("reach_position target_A", lambda m: m.get("at_target", False), priority=1)
])
env.add_agent(agent)

# Run simulation
for _ in range(100):
    env.step()
```

## Summary

AI agents provide the intelligence layer for robotic systems:

- Different agent types offer different capabilities
- The perception-decision-action loop is fundamental
- Model-based agents can reason about unobserved state
- Goal-based agents plan to achieve objectives
- Learning agents improve through experience

<div className="key-takeaways">

#### ✅ Key Takeaways

- **Agents** perceive, decide, and act autonomously
- **Reflex agents** are simple but limited
- **Model-based agents** maintain world state
- **Goal-based agents** plan to achieve objectives
- **Learning agents** improve over time

**Next Chapter**: [Agent Architectures →](/docs/ai-agent-integration/agent-architectures)

</div>



