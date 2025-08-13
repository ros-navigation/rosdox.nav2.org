---
layout: default
title: "CostmapFilterInfo Message"
permalink: /msgs/kilted/costmapfilterinfo.html
---

# CostmapFilterInfo Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

Filter information for costmap layers and processing

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `type` | `uint8` | Type of plugin used (keepout filter, speed limit in m/s, speed limit in percent, etc...) 0: keepout/lanes filter 1: speed limit filter in % of maximum speed 2: speed limit filter in absolute values (m/s) |
| `filter_mask_topic` | `string` | Name of filter mask topic |
| `base` | `float32` | Multiplier base offset and multiplier coefficient for conversion of OccGrid. Used to convert OccupancyGrid data values to filter space values. data -> into some other number space: space = data * multiplier + base |
| `multiplier` | `float32` | Floating point numerical value |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import CostmapFilterInfo

class CostmapFilterInfoPublisher(Node):
    def __init__(self):
        super().__init__('costmapfilterinfo_publisher')
        self.publisher = self.create_publisher(CostmapFilterInfo, 'costmapfilterinfo', 10)
        
    def publish_message(self):
        msg = CostmapFilterInfo()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        # Set msg.type as needed
        msg.filter_mask_topic = 'example_value'
        msg.base = 0.0
        msg.multiplier = 0.0
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"

class CostmapFilterInfoPublisher : public rclcpp::Node
{
public:
    CostmapFilterInfoPublisher() : Node("costmapfilterinfo_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::CostmapFilterInfo>("costmapfilterinfo", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::CostmapFilterInfo();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        // Set msg.type as needed
        msg.filter_mask_topic = "example_value";
        msg.base = 0.0;
        msg.multiplier = 0.0;
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;
};
```

## Related Messages

- [All Costmap Messages](/kilted/msgs/index.html#costmap-messages)
- [Message API Overview](/kilted/msgs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
