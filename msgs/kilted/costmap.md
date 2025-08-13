---
layout: default
title: "Costmap Message"
permalink: /msgs/kilted/costmap.html
---

# Costmap Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

Occupancy grid representation for navigation planning with cost values

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | This represents a 2-D grid map, in which each cell has an associated cost |
| `metadata` | `CostmapMetaData` | MetaData for the map |
| `data` | `uint8[]` | The cost data, in row-major order, starting with (0,0). |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Costmap

class CostmapPublisher(Node):
    def __init__(self):
        super().__init__('costmap_publisher')
        self.publisher = self.create_publisher(Costmap, 'costmap', 10)
        
    def publish_message(self):
        msg = Costmap()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        # Set msg.metadata as needed
        msg.data = []  # Fill array as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap.hpp"

class CostmapPublisher : public rclcpp::Node
{
public:
    CostmapPublisher() : Node("costmap_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::Costmap>("costmap", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::Costmap();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        // Set msg.metadata as needed
        // Fill msg.data array as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::Costmap>::SharedPtr publisher_;
};
```

## Related Messages

- [All Costmap Messages](/kilted/msgs/index.html#costmap-messages)
- [Message API Overview](/kilted/msgs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
