---
sidebar_label: "Sensor Simulation"
sidebar_position: 3
title: "Sensor Simulation: LiDAR, Depth Cameras, and IMU"
description: "Implementation of simulated sensor data streams"
---

# Sensor Simulation: LiDAR, Depth Cameras, and IMU

:::tip Learning Objective
Implement realistic sensor simulations for LiDAR, depth cameras, and IMU in Gazebo.
:::

## Sensor Overview

```mermaid
graph TB
    subgraph "Simulated Sensors"
        LIDAR[LiDAR<br/>2D/3D Point Cloud]
        DEPTH[Depth Camera<br/>RGB-D Images]
        IMU[IMU<br/>Orientation/Acceleration]
    end
    
    subgraph "ROS 2 Topics"
        SCAN[/scan]
        POINTS[/points]
        RGBD[/camera/depth]
        IMUDATA[/imu/data]
    end
    
    LIDAR --> SCAN
    LIDAR --> POINTS
    DEPTH --> RGBD
    IMU --> IMUDATA
```

## LiDAR Sensor Configuration

```xml
<!-- Add to URDF/SDF model -->
<gazebo reference="lidar_link">
  <sensor type="ray" name="lidar_sensor">
    <pose>0 0 0 0 0 0</pose>
    <visualize>true</visualize>
    <update_rate>10</update_rate>
    
    <ray>
      <scan>
        <horizontal>
          <samples>360</samples>
          <resolution>1</resolution>
          <min_angle>-3.14159</min_angle>
          <max_angle>3.14159</max_angle>
        </horizontal>
      </scan>
      <range>
        <min>0.2</min>
        <max>10.0</max>
        <resolution>0.01</resolution>
      </range>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.01</stddev>
      </noise>
    </ray>
    
    <plugin name="gazebo_ros_lidar" filename="libgazebo_ros_ray_sensor.so">
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=scan</remapping>
      </ros>
      <output_type>sensor_msgs/LaserScan</output_type>
      <frame_name>lidar_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Depth Camera Configuration

```xml
<gazebo reference="camera_link">
  <sensor type="depth" name="depth_camera">
    <update_rate>30</update_rate>
    
    <camera>
      <horizontal_fov>1.047</horizontal_fov>
      <image>
        <width>640</width>
        <height>480</height>
        <format>R8G8B8</format>
      </image>
      <depth_camera>
        <output>depths</output>
      </depth_camera>
      <clip>
        <near>0.1</near>
        <far>10.0</far>
      </clip>
      <noise>
        <type>gaussian</type>
        <mean>0.0</mean>
        <stddev>0.007</stddev>
      </noise>
    </camera>
    
    <plugin name="depth_camera_plugin" filename="libgazebo_ros_camera.so">
      <ros>
        <namespace>/camera</namespace>
      </ros>
      <camera_name>depth</camera_name>
      <frame_name>camera_link</frame_name>
      <hack_baseline>0.07</hack_baseline>
      <min_depth>0.1</min_depth>
      <max_depth>10.0</max_depth>
    </plugin>
  </sensor>
</gazebo>
```

## IMU Sensor Configuration

```xml
<gazebo reference="imu_link">
  <sensor type="imu" name="imu_sensor">
    <always_on>true</always_on>
    <update_rate>100</update_rate>
    
    <imu>
      <angular_velocity>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.0002</stddev>
          </noise>
        </z>
      </angular_velocity>
      <linear_acceleration>
        <x>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </x>
        <y>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </y>
        <z>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>0.017</stddev>
          </noise>
        </z>
      </linear_acceleration>
    </imu>
    
    <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
      <ros>
        <namespace>/</namespace>
        <remapping>~/out:=imu/data</remapping>
      </ros>
      <frame_name>imu_link</frame_name>
    </plugin>
  </sensor>
</gazebo>
```

## Sensor Processing Node

```python
#!/usr/bin/env python3
"""Process simulated sensor data for obstacle detection."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Image, Imu
from cv_bridge import CvBridge
import numpy as np


class SensorProcessor(Node):
    """Process LiDAR, camera, and IMU for obstacle sensing."""
    
    def __init__(self):
        super().__init__('sensor_processor')
        
        self.cv_bridge = CvBridge()
        
        # Subscribers
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth/image_raw', self.depth_callback, 10)
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10)
        
        self.get_logger().info('Sensor processor ready')
    
    def scan_callback(self, msg: LaserScan):
        """Detect obstacles from LiDAR."""
        ranges = np.array(msg.ranges)
        valid = ranges[(ranges > msg.range_min) & (ranges < msg.range_max)]
        
        if len(valid) > 0:
            min_dist = np.min(valid)
            min_idx = np.argmin(ranges)
            angle = msg.angle_min + min_idx * msg.angle_increment
            
            if min_dist < 1.0:
                self.get_logger().warn(
                    f'OBSTACLE: {min_dist:.2f}m at {np.degrees(angle):.1f}°'
                )
    
    def depth_callback(self, msg: Image):
        """Process depth image for obstacles."""
        depth = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='32FC1')
        
        # Check center region
        h, w = depth.shape
        center = depth[h//3:2*h//3, w//3:2*w//3]
        min_depth = np.nanmin(center)
        
        if min_depth < 0.5:
            self.get_logger().warn(f'CLOSE OBJECT: {min_depth:.2f}m')
    
    def imu_callback(self, msg: Imu):
        """Monitor robot orientation."""
        # Check for tipping
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        
        # Gravity should be mostly in Z
        horizontal_accel = np.sqrt(ax**2 + ay**2)
        if horizontal_accel > 3.0:
            self.get_logger().error('ROBOT TIPPING!')


def main():
    rclpy.init()
    node = SensorProcessor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
```

:::tip Module 2 Deliverable
✅ You now have a simulation environment where your robot can:
- Detect walls using LiDAR
- Sense obstacles with depth camera
- Monitor orientation with IMU

**Next Module:** [The AI-Robot Brain (NVIDIA Isaac) →](/docs/ai-robot-brain/isaac-sim)
:::


