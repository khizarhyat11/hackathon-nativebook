# ROS 2 Module Testing

This directory contains testing specifications for the ROS 2 Book Module.

## Test Categories

1. **Code Example Tests** - Verify all code examples run correctly
2. **Content Accuracy Tests** - Verify claims against official documentation
3. **URDF Validation Tests** - Verify URDF models validate correctly

## Test Procedures

### Code Example Verification

```bash
# Navigate to example directory
cd examples/ros2/publisher_subscriber/

# Run publisher in terminal 1
python3 minimal_publisher.py

# Run subscriber in terminal 2
python3 minimal_subscriber.py

# Verify messages are received
```

### URDF Validation

```bash
# Check URDF syntax
check_urdf humanoid_robot.urdf

# Output should show no errors
```
