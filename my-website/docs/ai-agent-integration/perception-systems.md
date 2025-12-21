---
sidebar_position: 3
title: Perception Systems
description: How robots perceive and understand their environment
---

# Perception Systems

<div className="learning-objectives">

#### 🎯 Learning Objectives

By the end of this chapter, you will be able to:
- Process visual data for object detection
- Implement sensor fusion for robust perception
- Create 3D world representations
- Apply deep learning for robot perception

</div>

## The Perception Pipeline

```python
from dataclasses import dataclass
from typing import List, Dict
import numpy as np

@dataclass
class PerceptionResult:
    """Output of perception pipeline."""
    objects: List[Dict]
    scene_geometry: np.ndarray
    confidence: float


class PerceptionPipeline:
    """Complete perception system for a robot."""
    
    def process(self, rgb: np.ndarray, depth: np.ndarray) -> PerceptionResult:
        """Run full perception pipeline."""
        objects = self.detect_objects(rgb)
        point_cloud = self.depth_to_pointcloud(depth)
        objects = self.associate_depth(objects, depth)
        
        return PerceptionResult(
            objects=objects,
            scene_geometry=point_cloud,
            confidence=self.compute_confidence(objects)
        )
    
    def depth_to_pointcloud(self, depth: np.ndarray) -> np.ndarray:
        """Convert depth image to 3D point cloud."""
        fx, fy = 525.0, 525.0
        cx, cy = 319.5, 239.5
        
        height, width = depth.shape
        u, v = np.meshgrid(np.arange(width), np.arange(height))
        
        z = depth
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        
        return np.stack([x, y, z], axis=-1).reshape(-1, 3)
```

## Object Detection

```python
class ObjectDetector:
    """Object detection using deep learning."""
    
    def __init__(self, confidence_threshold: float = 0.5):
        self.confidence_threshold = confidence_threshold
    
    def detect(self, image: np.ndarray) -> List[Dict]:
        """Detect objects in image."""
        # Example output format
        return [
            {
                "class_name": "cup",
                "confidence": 0.92,
                "bbox": [100, 150, 180, 250],
                "center": [140, 200],
            }
        ]
```

## Sensor Fusion

```python
class SensorFusion:
    """Fuse data from multiple sensors."""
    
    def fuse_point_clouds(self, clouds: Dict[str, np.ndarray]) -> np.ndarray:
        """Fuse point clouds from multiple depth sensors."""
        all_points = [cloud for cloud in clouds.values()]
        return np.vstack(all_points) if all_points else np.array([])
    
    def kalman_fuse(self, visual_pos: np.ndarray, 
                    imu_pos: np.ndarray,
                    v_uncertainty: float,
                    i_uncertainty: float) -> np.ndarray:
        """Fuse positions using Kalman approach."""
        total = v_uncertainty + i_uncertainty
        w_v = i_uncertainty / total
        w_i = v_uncertainty / total
        return w_v * visual_pos + w_i * imu_pos
```

## Scene Understanding

```python
class SceneUnderstanding:
    """High-level scene understanding."""
    
    def build_representation(self, objects: List[Dict]) -> Dict:
        """Build structured scene representation."""
        return {
            "objects": {f"{o['class_name']}_{i}": o 
                       for i, o in enumerate(objects)},
            "relations": self.compute_relations(objects)
        }
    
    def compute_relations(self, objects: List[Dict]) -> List[Dict]:
        """Compute spatial relations between objects."""
        relations = []
        for i, obj1 in enumerate(objects):
            for obj2 in objects[i+1:]:
                rel = self.get_relation(obj1, obj2)
                if rel:
                    relations.append(rel)
        return relations
```

<div className="key-takeaways">

#### ✅ Key Takeaways

- Modern perception uses **deep learning** for visual understanding
- **Sensor fusion** improves robustness and accuracy
- **Scene representations** enable reasoning about the environment

**Next Chapter**: [Decision Making →](/docs/ai-agent-integration/decision-making)

</div>



