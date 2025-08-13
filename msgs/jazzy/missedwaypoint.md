---
layout: default
title: "MissedWaypoint Message"
permalink: /msgs/jazzy/missedwaypoint.html
---

# MissedWaypoint Message

**Package:** `nav2_msgs`  
**Category:** Waypoint Messages

Nav2 message for robotic navigation and behavior control

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `index` | `uint32` | Integer numerical value |
| `goal` | `geometry_msgs/PoseStamped` | Pose with header information (frame_id and timestamp) |
| `error_code` | `uint16` | Numeric error code if operation failed |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import MissedWaypoint

class MissedWaypointPublisher(Node):
    def __init__(self):
        super().__init__('missedwaypoint_publisher')
        self.publisher = self.create_publisher(MissedWaypoint, 'missedwaypoint', 10)
        
    def publish_message(self):
        msg = MissedWaypoint()
        msg.index = 0
        # Set msg.goal as needed
        msg.error_code = 0
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/missed_waypoint.hpp"

class MissedWaypointPublisher : public rclcpp::Node
{
public:
    MissedWaypointPublisher() : Node("missedwaypoint_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::MissedWaypoint>("missedwaypoint", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::MissedWaypoint();
        msg.index = 0;
        // Set msg.goal as needed
        msg.error_code = 0;
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::MissedWaypoint>::SharedPtr publisher_;
};
```

## Related Messages

- [All Waypoint Messages](/jazzy/msgs/index.html#waypoint-messages)
- [Message API Overview](/jazzy/msgs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
