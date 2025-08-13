---
layout: default
title: "RouteEdge Message"
permalink: /msgs/kilted/routeedge.html
---

# RouteEdge Message

**Package:** `nav2_msgs`  
**Category:** Routing Messages

Connection between route nodes with traversal cost

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `edgeid` | `uint16` | Integer numerical value |
| `start` | `geometry_msgs/Point` | Message field - see Nav2 documentation for specific usage details |
| `end` | `geometry_msgs/Point` | Message field - see Nav2 documentation for specific usage details |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import RouteEdge

class RouteEdgePublisher(Node):
    def __init__(self):
        super().__init__('routeedge_publisher')
        self.publisher = self.create_publisher(RouteEdge, 'routeedge', 10)
        
    def publish_message(self):
        msg = RouteEdge()
        msg.edgeid = 0
        # Set msg.start as needed
        # Set msg.end as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/route_edge.hpp"

class RouteEdgePublisher : public rclcpp::Node
{
public:
    RouteEdgePublisher() : Node("routeedge_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::RouteEdge>("routeedge", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::RouteEdge();
        msg.edgeid = 0;
        // Set msg.start as needed
        // Set msg.end as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::RouteEdge>::SharedPtr publisher_;
};
```

## Related Messages

- [All Routing Messages](/kilted/msgs/index.html#routing-messages)
- [Message API Overview](/kilted/msgs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
