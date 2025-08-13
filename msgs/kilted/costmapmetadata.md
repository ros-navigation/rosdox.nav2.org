---
layout: default
title: "CostmapMetaData Message"
permalink: /msgs/kilted/costmapmetadata.html
---

# CostmapMetaData Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

Metadata information for costmap including dimensions and resolution

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `map_load_time` | `builtin_interfaces/Time` | This hold basic information about the characteristics of the Costmap The time at which the static map was loaded |
| `update_time` | `builtin_interfaces/Time` | The time of the last update to costmap |
| `layer` | `string` | The corresponding layer name |
| `resolution` | `float32` | The map resolution [m/cell] |
| `size_x` | `uint32` | Number of cells in the horizontal direction |
| `size_y` | `uint32` | Number of cells in the vertical direction |
| `origin` | `geometry_msgs/Pose` | The origin of the costmap [m, m, rad]. This is the real-world pose of the cell (0,0) in the map. |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import CostmapMetaData

class CostmapMetaDataPublisher(Node):
    def __init__(self):
        super().__init__('costmapmetadata_publisher')
        self.publisher = self.create_publisher(CostmapMetaData, 'costmapmetadata', 10)
        
    def publish_message(self):
        msg = CostmapMetaData()
        # Set msg.map_load_time as needed
        # Set msg.update_time as needed
        msg.layer = 'example_value'
        msg.resolution = 0.0
        msg.size_x = 0
        msg.size_y = 0
        # Set msg.origin as needed
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/costmap_meta_data.hpp"

class CostmapMetaDataPublisher : public rclcpp::Node
{
public:
    CostmapMetaDataPublisher() : Node("costmapmetadata_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::CostmapMetaData>("costmapmetadata", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::CostmapMetaData();
        // Set msg.map_load_time as needed
        // Set msg.update_time as needed
        msg.layer = "example_value";
        msg.resolution = 0.0;
        msg.size_x = 0;
        msg.size_y = 0;
        // Set msg.origin as needed
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::CostmapMetaData>::SharedPtr publisher_;
};
```

## Related Messages

- [All Costmap Messages](/kilted/msgs/index.html#costmap-messages)
- [Message API Overview](/kilted/msgs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
