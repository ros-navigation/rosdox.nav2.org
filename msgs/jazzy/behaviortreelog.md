---
layout: default
title: "BehaviorTreeLog Message"
permalink: /msgs/jazzy/behaviortreelog.html
---

# BehaviorTreeLog Message

**Package:** `nav2_msgs`  
**Category:** Behavior Tree Messages

Logging information from behavior tree execution

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | `builtin_interfaces/Time` | ROS time that this log message was sent. |
| `event_log` | `BehaviorTreeStatusChange[]` | Array of BehaviorTreeStatusChange values |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeLog

class BehaviorTreeLogPublisher(Node):
    def __init__(self):
        super().__init__('behaviortreelog_publisher')
        self.publisher = self.create_publisher(BehaviorTreeLog, 'behaviortreelog', 10)
        
    def publish_message(self):
        msg = BehaviorTreeLog()
        # Set msg.timestamp as needed
        msg.event_log = []  # Fill array as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_log.hpp"

class BehaviorTreeLogPublisher : public rclcpp::Node
{
public:
    BehaviorTreeLogPublisher() : Node("behaviortreelog_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::BehaviorTreeLog>("behaviortreelog", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::BehaviorTreeLog();
        // Set msg.timestamp as needed
        // Fill msg.event_log array as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeLog>::SharedPtr publisher_;
};
```

## Related Messages

- [All Behavior Tree Messages](/jazzy/msgs/index.html#behavior-tree-messages)
- [Message API Overview](/jazzy/msgs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
