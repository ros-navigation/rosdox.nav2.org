---
layout: default
title: "EdgeCost Message"
permalink: /msgs/humble/edgecost.html
---

# EdgeCost Message

**Package:** `nav2_msgs`  
**Category:** Routing Messages

Cost information for route graph edges

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `edgeid` | `uint16` | Edge cost to use with nav2_msgs/srv/DynamicEdges to adjust route edge costs |
| `cost` | `float32` | Cost for traversing the edge |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import EdgeCost

class EdgeCostPublisher(Node):
    def __init__(self):
        super().__init__('edgecost_publisher')
        self.publisher = self.create_publisher(EdgeCost, 'edgecost', 10)
        
    def publish_message(self):
        msg = EdgeCost()
        msg.edgeid = 0
        msg.cost = 0.0
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/edge_cost.hpp"

class EdgeCostPublisher : public rclcpp::Node
{
public:
    EdgeCostPublisher() : Node("edgecost_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::EdgeCost>("edgecost", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::EdgeCost();
        msg.edgeid = 0;
        msg.cost = 0.0;
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::EdgeCost>::SharedPtr publisher_;
};
```

## Related Messages

- [All Routing Messages](/humble/msgs/index.html#routing-messages)
- [Message API Overview](/humble/msgs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
