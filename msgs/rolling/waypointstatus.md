---
layout: default
title: "WaypointStatus Message"
permalink: /msgs/rolling/waypointstatus.html
---

# WaypointStatus Message

**Package:** `nav2_msgs`  
**Category:** Waypoint Messages

Status information for waypoint navigation tasks

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `waypoint_status` | `uint8` | Status of the waypoint (0=PENDING, 1=COMPLETED, 2=SKIPPED, 3=FAILED) |
| `waypoint_index` | `uint32` | Index of the current waypoint in sequence |
| `waypoint_pose` | `geometry_msgs/PoseStamped` | Pose of the waypoint |
| `error_code` | `uint16` | Numeric error code if operation failed |
| `error_msg` | `string` | Human-readable error message |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import WaypointStatus

class WaypointStatusPublisher(Node):
    def __init__(self):
        super().__init__('waypointstatus_publisher')
        self.publisher = self.create_publisher(WaypointStatus, 'waypointstatus', 10)
        
    def publish_message(self):
        msg = WaypointStatus()
        # Set msg.waypoint_status as needed
        msg.waypoint_index = 0
        # Set msg.waypoint_pose as needed
        msg.error_code = 0
        msg.error_msg = 'example_value'
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/waypoint_status.hpp"

class WaypointStatusPublisher : public rclcpp::Node
{
public:
    WaypointStatusPublisher() : Node("waypointstatus_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::WaypointStatus>("waypointstatus", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::WaypointStatus();
        // Set msg.waypoint_status as needed
        msg.waypoint_index = 0;
        // Set msg.waypoint_pose as needed
        msg.error_code = 0;
        msg.error_msg = "example_value";
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::WaypointStatus>::SharedPtr publisher_;
};
```

## Related Messages

- [All Waypoint Messages](/rolling/msgs/index.html#waypoint-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
