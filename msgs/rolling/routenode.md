---
layout: default
title: "RouteNode Message"
permalink: /msgs/rolling/routenode.html
---

# RouteNode Message

**Package:** `nav2_msgs`  
**Category:** Routing Messages

Individual waypoint node in a route graph

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `nodeid` | `uint16` | ID of the node in the graph |
| `position` | `geometry_msgs/Point` | Message field - see Nav2 documentation for specific usage details |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import RouteNode

class RouteNodePublisher(Node):
    def __init__(self):
        super().__init__('routenode_publisher')
        self.publisher = self.create_publisher(RouteNode, 'routenode', 10)
        
    def publish_message(self):
        msg = RouteNode()
        msg.nodeid = 0
        # Set msg.position as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/route_node.hpp"

class RouteNodePublisher : public rclcpp::Node
{
public:
    RouteNodePublisher() : Node("routenode_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::RouteNode>("routenode", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::RouteNode();
        msg.nodeid = 0;
        // Set msg.position as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::RouteNode>::SharedPtr publisher_;
};
```

## Related Messages

- [All Routing Messages](/rolling/msgs/index.html#routing-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
