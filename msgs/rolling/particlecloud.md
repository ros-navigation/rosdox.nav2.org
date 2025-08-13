---
layout: default
title: "ParticleCloud Message"
permalink: /msgs/rolling/particlecloud.html
---

# ParticleCloud Message

**Package:** `nav2_msgs`  
**Category:** Localization Messages

Collection of particles representing pose distribution in particle filter

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | This represents a particle cloud containing particle poses and weights |
| `particles` | `Particle[]` | Array of particles in the cloud |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import ParticleCloud

class ParticleCloudPublisher(Node):
    def __init__(self):
        super().__init__('particlecloud_publisher')
        self.publisher = self.create_publisher(ParticleCloud, 'particlecloud', 10)
        
    def publish_message(self):
        msg = ParticleCloud()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.particles = []  # Fill array as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/particle_cloud.hpp"

class ParticleCloudPublisher : public rclcpp::Node
{
public:
    ParticleCloudPublisher() : Node("particlecloud_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::ParticleCloud>("particlecloud", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::ParticleCloud();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        // Fill msg.particles array as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::ParticleCloud>::SharedPtr publisher_;
};
```

## Related Messages

- [All Localization Messages](/rolling/msgs/index.html#localization-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
