---
layout: default
title: "CostmapUpdate Message"
permalink: /msgs/kilted/costmapupdate.html
---

# CostmapUpdate Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

Update message containing changed regions of a costmap

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Update msg for Costmap containing the modified part of Costmap |
| `x` | `uint32` | Integer numerical value |
| `y` | `uint32` | Integer numerical value |
| `size_x` | `uint32` | Integer numerical value |
| `size_y` | `uint32` | Integer numerical value |
| `data` | `uint8[]` | The cost data, in row-major order, starting with (x,y) from 0-255 in Costmap format rather than OccupancyGrid 0-100. |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import CostmapUpdate

class CostmapUpdatePublisher(Node):
    def __init__(self):
        super().__init__('costmapupdate_publisher')
        self.publisher = self.create_publisher(CostmapUpdate, 'costmapupdate', 10)
        
    def publish_message(self):
        msg = CostmapUpdate()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.x = 0
        msg.y = 0
        msg.size_x = 0
        msg.size_y = 0
        msg.data = []  # Fill array as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap_update.hpp"

class CostmapUpdatePublisher : public rclcpp::Node
{
public:
    CostmapUpdatePublisher() : Node("costmapupdate_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::CostmapUpdate>("costmapupdate", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::CostmapUpdate();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.x = 0;
        msg.y = 0;
        msg.size_x = 0;
        msg.size_y = 0;
        // Fill msg.data array as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::CostmapUpdate>::SharedPtr publisher_;
};
```

## Related Messages

- [All Costmap Messages](/kilted/msgs/index.html#costmap-messages)
- [Message API Overview](/kilted/msgs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
