---
sidebar_label: "Nav2 Navigation"
sidebar_position: 3
title: "Nav2 Navigation for Bipedal Path Planning"
description: "Configure Navigation2 stack for humanoid robot navigation"
---

# Nav2 Navigation for Bipedal Path Planning

:::tip Learning Objective
Configure Nav2 for autonomous navigation, enabling your robot to move from point A to point B.
:::

## Nav2 Architecture

```mermaid
graph TB
    subgraph "Nav2 Stack"
        GOAL[Goal Pose] --> BT[Behavior Tree]
        BT --> PLANNER[Global Planner]
        BT --> CONTROLLER[Local Controller]
        
        MAP[Map Server] --> PLANNER
        COSTMAP[Costmap 2D] --> PLANNER
        COSTMAP --> CONTROLLER
        
        CONTROLLER --> CMD[/cmd_vel]
    end
    
    SENSORS[Sensors] --> COSTMAP
    SLAM[SLAM] --> MAP
```

## Nav2 Parameters Configuration

```yaml
# nav2_params.yaml
controller_server:
  ros__parameters:
    controller_frequency: 20.0
    
    # DWB Controller for differential drive
    FollowPath:
      plugin: "dwb_core::DWBLocalPlanner"
      min_vel_x: 0.0
      max_vel_x: 0.5
      max_vel_theta: 1.0
      min_speed_xy: 0.0
      max_speed_xy: 0.5
      acc_lim_x: 2.5
      acc_lim_theta: 3.2
      decel_lim_x: -2.5

planner_server:
  ros__parameters:
    expected_planner_frequency: 20.0
    
    GridBased:
      plugin: "nav2_navfn_planner/NavfnPlanner"
      tolerance: 0.5
      use_astar: true
      allow_unknown: true

costmap_common:
  robot_base_frame: base_link
  global_frame: map
  robot_radius: 0.3
  resolution: 0.05
  
  obstacle_layer:
    plugin: "nav2_costmap_2d::ObstacleLayer"
    enabled: True
    observation_sources: scan
    scan:
      topic: /scan
      data_type: LaserScan
      clearing: True
      marking: True
      
  inflation_layer:
    plugin: "nav2_costmap_2d::InflationLayer"
    cost_scaling_factor: 3.0
    inflation_radius: 0.55
```

## Navigation Action Client

```python
#!/usr/bin/env python3
"""Navigate robot from A to B using Nav2."""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math


class NavigationClient(Node):
    """Client for Nav2 navigation actions."""
    
    def __init__(self):
        super().__init__('navigation_client')
        
        self._action_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose'
        )
        
        self.get_logger().info('Navigation client ready')
    
    def navigate_to(self, x: float, y: float, yaw: float = 0.0):
        """Send navigation goal."""
        
        # Wait for action server
        self.get_logger().info('Waiting for Nav2...')
        self._action_client.wait_for_server()
        
        # Create goal
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        
        self.get_logger().info(f'Navigating to ({x}, {y})')
        
        # Send goal
        send_goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_callback
        )
        send_goal_future.add_done_callback(self._goal_response_callback)
    
    def _goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return
        
        self.get_logger().info('Goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)
    
    def _feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        remaining = feedback.distance_remaining
        self.get_logger().info(f'Distance remaining: {remaining:.2f}m')
    
    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Navigation complete!')


def main():
    rclpy.init()
    
    navigator = NavigationClient()
    
    # Navigate to point (3, 2) facing forward
    navigator.navigate_to(3.0, 2.0, 0.0)
    
    rclpy.spin(navigator)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

## Waypoint Navigation

```python
#!/usr/bin/env python3
"""Navigate through multiple waypoints."""

from typing import List, Tuple

class WaypointNavigator(NavigationClient):
    """Navigate through a sequence of waypoints."""
    
    def __init__(self):
        super().__init__()
        self.waypoints: List[Tuple[float, float]] = []
        self.current_waypoint = 0
    
    def set_waypoints(self, waypoints: List[Tuple[float, float]]):
        """Set list of waypoints to visit."""
        self.waypoints = waypoints
        self.current_waypoint = 0
    
    def start_patrol(self):
        """Begin navigating through waypoints."""
        if self.waypoints:
            x, y = self.waypoints[0]
            self.navigate_to(x, y)
    
    def _result_callback(self, future):
        """Move to next waypoint when current one reached."""
        super()._result_callback(future)
        
        self.current_waypoint += 1
        
        if self.current_waypoint < len(self.waypoints):
            x, y = self.waypoints[self.current_waypoint]
            self.navigate_to(x, y)
        else:
            self.get_logger().info('All waypoints reached!')


# Usage
navigator = WaypointNavigator()
navigator.set_waypoints([
    (1.0, 0.0),
    (2.0, 1.0),
    (3.0, 0.0),
    (0.0, 0.0)  # Return home
])
navigator.start_patrol()
```

:::tip Module 3 Deliverable
✅ You now have a robot that can:
1. Generate synthetic training data with Isaac Sim
2. Build maps using Visual SLAM
3. Navigate autonomously from point A to point B

**Next Module:** [Vision-Language-Action (VLA) →](/docs/vision-language-action/voice-pipeline)
:::


