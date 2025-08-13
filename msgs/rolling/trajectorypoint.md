---
layout: default
title: "TrajectoryPoint Message"
permalink: /msgs/rolling/trajectorypoint.html
---

# TrajectoryPoint Message

**Package:** `nav2_msgs`  
**Category:** Planning Messages

Single point in a trajectory with pose, velocity, and timing

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `time_from_start` | `builtin_interfaces/Duration` | Trajectory point state Desired time from the trajectory start to arrive at this trajectory sample. |
| `pose` | `geometry_msgs/Pose` | Pose of the trajectory sample. |
| `velocity` | `geometry_msgs/Twist` | Velocity of the trajectory sample. |
| `acceleration` | `geometry_msgs/Accel` | Acceleration of the trajectory (optional). |
| `effort` | `geometry_msgs/Wrench` | Force/Torque to apply at trajectory sample (optional). |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import TrajectoryPoint

class TrajectoryPointPublisher(Node):
    def __init__(self):
        super().__init__('trajectorypoint_publisher')
        self.publisher = self.create_publisher(TrajectoryPoint, 'trajectorypoint', 10)
        
    def publish_message(self):
        msg = TrajectoryPoint()
        # Set msg.time_from_start as needed
        # Set msg.pose as needed
        # Set msg.velocity as needed
        # Set msg.acceleration as needed
        # Set msg.effort as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/trajectory_point.hpp"

class TrajectoryPointPublisher : public rclcpp::Node
{
public:
    TrajectoryPointPublisher() : Node("trajectorypoint_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::TrajectoryPoint>("trajectorypoint", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::TrajectoryPoint();
        // Set msg.time_from_start as needed
        // Set msg.pose as needed
        // Set msg.velocity as needed
        // Set msg.acceleration as needed
        // Set msg.effort as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::TrajectoryPoint>::SharedPtr publisher_;
};
```

## Related Messages

- [All Planning Messages](/rolling/msgs/index.html#planning-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
