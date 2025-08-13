---
layout: default
title: "SpeedLimit Message"
permalink: /msgs/kilted/speedlimit.html
---

# SpeedLimit Message

**Package:** `nav2_msgs`  
**Category:** Control Messages

Speed limit information for navigation areas

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `percentage` | `bool` | Setting speed limit in percentage if true or in absolute values in false case |
| `speed_limit` | `float64` | Maximum allowed speed (in percent of maximum robot speed or in m/s depending on "percentage" value). When no-limit it is set to 0.0 |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import SpeedLimit

class SpeedLimitPublisher(Node):
    def __init__(self):
        super().__init__('speedlimit_publisher')
        self.publisher = self.create_publisher(SpeedLimit, 'speedlimit', 10)
        
    def publish_message(self):
        msg = SpeedLimit()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = True
        msg.speed_limit = 0.0
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/speed_limit.hpp"

class SpeedLimitPublisher : public rclcpp::Node
{
public:
    SpeedLimitPublisher() : Node("speedlimit_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::SpeedLimit>("speedlimit", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::SpeedLimit();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.percentage = true;
        msg.speed_limit = 0.0;
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::SpeedLimit>::SharedPtr publisher_;
};
```

## Related Messages

- [All Control Messages](/kilted/msgs/index.html#control-messages)
- [Message API Overview](/kilted/msgs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
