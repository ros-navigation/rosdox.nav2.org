---
layout: default
title: "CollisionDetectorState Message"
permalink: /msgs/rolling/collisiondetectorstate.html
---

# CollisionDetectorState Message

**Package:** `nav2_msgs`  
**Category:** Collision Messages

State information from collision detection systems

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `polygons` | `string[]` | Name of configured polygons |
| `detections` | `bool[]` | List of detections for each polygon |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import CollisionDetectorState

class CollisionDetectorStatePublisher(Node):
    def __init__(self):
        super().__init__('collisiondetectorstate_publisher')
        self.publisher = self.create_publisher(CollisionDetectorState, 'collisiondetectorstate', 10)
        
    def publish_message(self):
        msg = CollisionDetectorState()
        msg.polygons = []  # Fill array as needed
        msg.detections = []  # Fill array as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/collision_detector_state.hpp"

class CollisionDetectorStatePublisher : public rclcpp::Node
{
public:
    CollisionDetectorStatePublisher() : Node("collisiondetectorstate_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::CollisionDetectorState>("collisiondetectorstate", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::CollisionDetectorState();
        // Fill msg.polygons array as needed
        // Fill msg.detections array as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::CollisionDetectorState>::SharedPtr publisher_;
};
```

## Related Messages

- [All Collision Messages](/rolling/msgs/index.html#collision-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
