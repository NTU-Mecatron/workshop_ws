# Mecatron Software Training Workshop

This repository contains the code and resources for the Mecatron software training workshop. It's a comprehensive ROS 2 workspace designed to teach robotics software development fundamentals using ROS 2 Humble and ament_cmake build system.

## Repository Structure

The workspace contains several key packages:
- **workshop_main** - Core implementation packages with example nodes and applications for publisher and subscriber patterns
- **workshop_service** - Core implementation packages with example nodes and applications for service and client patterns

## Getting Started

This workspace is organized for hands-on training and demonstration purposes, helping participants learn and practice robotics software development using modern ROS 2 practices and ament_cmake build tools.

## Notes to Avoid Confusion
### Publishers

In the `minimal_publisher.py` file, we created a timer to call the timer_callback function every `0.5` seconds. The function publishes a `Float32` message with a value of `0.5` to the topic `/mavros/setpoint_velocity/cmd_vel_unstamped/x` at line 18, by:
```python
self.publisher_.publish(msg)
```

**Important Notes:**

* The publish function does not have to be inside the timer callback.
* You can call it anywhere: in a subscriber callback, service callback, or even in the class constructor (__init__).
* The timer is just a convenient way to call a function periodically.