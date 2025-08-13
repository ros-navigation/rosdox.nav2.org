---
layout: default
title: "Particle Message"
permalink: /msgs/rolling/particle.html
---

# Particle Message

**Package:** `nav2_msgs`  
**Category:** Localization Messages

Single particle hypothesis with pose and weight for particle filter localization

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `pose` | `geometry_msgs/Pose` | This represents an individual particle with weight produced by a particle filter |
| `weight` | `float64` | Probability weight of this particle hypothesis |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Particle

class ParticlePublisher(Node):
    def __init__(self):
        super().__init__('particle_publisher')
        self.publisher = self.create_publisher(Particle, 'particle', 10)
        
    def publish_message(self):
        msg = Particle()
        # Set msg.pose as needed
        msg.weight = 0.0
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/particle.hpp"

class ParticlePublisher : public rclcpp::Node
{
public:
    ParticlePublisher() : Node("particle_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::Particle>("particle", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::Particle();
        // Set msg.pose as needed
        msg.weight = 0.0;
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::Particle>::SharedPtr publisher_;
};
```

## Related Messages

- [All Localization Messages](/rolling/msgs/index.html#localization-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
