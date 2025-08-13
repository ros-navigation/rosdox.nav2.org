---
layout: default
title: "CollisionMonitorState Message"
permalink: /msgs/kilted/collisionmonitorstate.html
---

# CollisionMonitorState Message

**Package:** `nav2_msgs`  
**Category:** Collision Messages

Monitoring state for collision avoidance systems

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `DO_NOTHING` = `0 # No action` | `uint8` | Action type for robot in Collision Monitor. No action |
| `STOP` = `1 # Stop the robot` | `uint8` | Stop the robot |
| `SLOWDOWN` = `2 # Slowdown in percentage from current operating speed` | `uint8` | Slowdown in percentage from current operating speed |
| `APPROACH` = `3 # Keep constant time interval before collision` | `uint8` | Keep constant time interval before collision |
| `LIMIT` = `4 # Sets a limit of velocities if pts in range` | `uint8` | Sets a limit of velocities if pts in range |
| `action_type` | `uint8` | Message field - see Nav2 documentation for specific usage details |
| `polygon_name` | `string` | Name of triggered polygon |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import CollisionMonitorState

class CollisionMonitorStatePublisher(Node):
    def __init__(self):
        super().__init__('collisionmonitorstate_publisher')
        self.publisher = self.create_publisher(CollisionMonitorState, 'collisionmonitorstate', 10)
        
    def publish_message(self):
        msg = CollisionMonitorState()
        # Set msg.action_type as needed
        msg.polygon_name = 'example_value'
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/collision_monitor_state.hpp"

class CollisionMonitorStatePublisher : public rclcpp::Node
{
public:
    CollisionMonitorStatePublisher() : Node("collisionmonitorstate_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::CollisionMonitorState>("collisionmonitorstate", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::CollisionMonitorState();
        // Set msg.action_type as needed
        msg.polygon_name = "example_value";
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::CollisionMonitorState>::SharedPtr publisher_;
};
```

## Related Messages

- [All Collision Messages](/kilted/msgs/index.html#collision-messages)
- [Message API Overview](/kilted/msgs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
