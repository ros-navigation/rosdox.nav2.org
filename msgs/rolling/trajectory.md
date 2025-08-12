---
layout: default
title: "Trajectory Message"
permalink: /msgs/rolling/trajectory.html
---

# Trajectory Message

**Package:** `nav2_msgs`  
**Category:** Planning Messages

Time-parameterized path with poses and velocities

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | An array of trajectory points that represents a trajectory for a robot to follow. Indicates the frame_id of the trajectory. |
| `points` | `TrajectoryPoint[]` | Array of trajectory points to follow. |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Trajectory

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher = self.create_publisher(Trajectory, 'trajectory', 10)
        
    def publish_message(self):
        msg = Trajectory()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.points = []  # Fill array as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/trajectory.hpp"

class TrajectoryPublisher : public rclcpp::Node
{
public:
    TrajectoryPublisher() : Node("trajectory_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::Trajectory>("trajectory", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::Trajectory();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        // Fill msg.points array as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::Trajectory>::SharedPtr publisher_;
};
```

## Related Messages

- [All Planning Messages](/rolling/msgs/index.html#planning-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
