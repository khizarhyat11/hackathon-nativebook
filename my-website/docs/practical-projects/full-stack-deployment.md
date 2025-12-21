---
sidebar_position: 4
title: Full-Stack Deployment
description: Deploying a complete Physical AI system
---

# Full-Stack Deployment

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will:
- Integrate all system components
- Deploy to real or simulated robots
- Monitor system performance
- Handle production considerations

</div>

## System Architecture

```python
from dataclasses import dataclass
from typing import Optional
import threading
import time

@dataclass
class SystemConfig:
    robot_type: str
    simulation: bool
    perception_rate: float = 30.0
    control_rate: float = 100.0


class PhysicalAISystem:
    """Complete Physical AI system integration."""
    
    def __init__(self, config: SystemConfig):
        self.config = config
        self.running = False
        
        # Initialize subsystems
        self.perception = None
        self.planner = None
        self.controller = None
        self.agent = None
    
    def start(self):
        """Start all subsystems."""
        self.running = True
        
        # Start threads
        threading.Thread(target=self._perception_loop, daemon=True).start()
        threading.Thread(target=self._control_loop, daemon=True).start()
        threading.Thread(target=self._agent_loop, daemon=True).start()
        
        print("System started!")
    
    def _perception_loop(self):
        period = 1.0 / self.config.perception_rate
        while self.running:
            # Process sensors
            start = time.time()
            # self.perception.update()
            elapsed = time.time() - start
            time.sleep(max(0, period - elapsed))
    
    def _control_loop(self):
        period = 1.0 / self.config.control_rate
        while self.running:
            # Send motor commands
            start = time.time()
            # self.controller.update()
            elapsed = time.time() - start
            time.sleep(max(0, period - elapsed))
    
    def _agent_loop(self):
        while self.running:
            # AI reasoning
            # self.agent.step()
            time.sleep(0.1)
    
    def stop(self):
        self.running = False
        print("System stopped.")


# Deployment script
def deploy_system():
    config = SystemConfig(
        robot_type="kuka_iiwa",
        simulation=True
    )
    
    system = PhysicalAISystem(config)
    system.start()
    
    try:
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        system.stop()
```

## Monitoring

```python
class SystemMonitor:
    """Monitor system health and performance."""
    
    def __init__(self):
        self.metrics = {}
    
    def log_metric(self, name: str, value: float):
        if name not in self.metrics:
            self.metrics[name] = []
        self.metrics[name].append((time.time(), value))
    
    def get_summary(self) -> dict:
        summary = {}
        for name, values in self.metrics.items():
            vals = [v[1] for v in values[-100:]]
            summary[name] = {
                "mean": sum(vals) / len(vals),
                "min": min(vals),
                "max": max(vals)
            }
        return summary
```

<div className="key-takeaways">

#### ✅ Key Takeaways

- **Integration** combines perception, planning, and control
- **Multi-threading** handles different update rates
- **Monitoring** ensures system health
- **Graceful shutdown** prevents hardware damage

🎓 **Congratulations!** You've completed the Physical AI & Humanoid Robotics curriculum!

</div>



