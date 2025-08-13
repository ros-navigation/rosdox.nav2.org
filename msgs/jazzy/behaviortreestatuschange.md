---
layout: default
title: "BehaviorTreeStatusChange Message"
permalink: /msgs/jazzy/behaviortreestatuschange.html
---

# BehaviorTreeStatusChange Message

**Package:** `nav2_msgs`  
**Category:** Behavior Tree Messages

Status change events from behavior tree nodes

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | `builtin_interfaces/Time` | internal behavior tree event timestamp. Typically this is wall clock time |
| `node_name` | `string` | String value or identifier |
| `uid` | `uint16` | unique ID for this node |
| `previous_status` | `string` | IDLE, RUNNING, SUCCESS or FAILURE |
| `current_status` | `string` | IDLE, RUNNING, SUCCESS or FAILURE |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import BehaviorTreeStatusChange

class BehaviorTreeStatusChangePublisher(Node):
    def __init__(self):
        super().__init__('behaviortreestatuschange_publisher')
        self.publisher = self.create_publisher(BehaviorTreeStatusChange, 'behaviortreestatuschange', 10)
        
    def publish_message(self):
        msg = BehaviorTreeStatusChange()
        # Set msg.timestamp as needed
        msg.node_name = 'example_value'
        msg.uid = 0
        msg.previous_status = 'example_value'
        msg.current_status = 'example_value'
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/behavior_tree_status_change.hpp"

class BehaviorTreeStatusChangePublisher : public rclcpp::Node
{
public:
    BehaviorTreeStatusChangePublisher() : Node("behaviortreestatuschange_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::BehaviorTreeStatusChange>("behaviortreestatuschange", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::BehaviorTreeStatusChange();
        // Set msg.timestamp as needed
        msg.node_name = "example_value";
        msg.uid = 0;
        msg.previous_status = "example_value";
        msg.current_status = "example_value";
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::BehaviorTreeStatusChange>::SharedPtr publisher_;
};
```

## Related Messages

- [All Behavior Tree Messages](/jazzy/msgs/index.html#behavior-tree-messages)
- [Message API Overview](/jazzy/msgs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
