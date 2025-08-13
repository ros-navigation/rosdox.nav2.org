---
layout: default
title: "Route Message"
permalink: /msgs/humble/route.html
---

# Route Message

**Package:** `nav2_msgs`  
**Category:** Routing Messages

Complete route plan with waypoints and metadata

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `route_cost` | `float32` | Floating point numerical value |
| `nodes` | `RouteNode[]` | ordered set of nodes of the route |
| `edges` | `RouteEdge[]` | ordered set of edges that connect nodes |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import Route

class RoutePublisher(Node):
    def __init__(self):
        super().__init__('route_publisher')
        self.publisher = self.create_publisher(Route, 'route', 10)
        
    def publish_message(self):
        msg = Route()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.route_cost = 0.0
        msg.nodes = []  # Fill array as needed
        msg.edges = []  # Fill array as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/route.hpp"

class RoutePublisher : public rclcpp::Node
{
public:
    RoutePublisher() : Node("route_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::Route>("route", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::Route();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        msg.route_cost = 0.0;
        // Fill msg.nodes array as needed
        // Fill msg.edges array as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::Route>::SharedPtr publisher_;
};
```

## Related Messages

- [All Routing Messages](/humble/msgs/index.html#routing-messages)
- [Message API Overview](/humble/msgs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
