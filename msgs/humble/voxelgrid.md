---
layout: default
title: "VoxelGrid Message"
permalink: /msgs/humble/voxelgrid.html
---

# VoxelGrid Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

3D voxel grid representation for obstacle detection and mapping

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `data` | `uint32[]` | Occupancy grid data as a flat array of cost values |
| `origin` | `geometry_msgs/Point32` | Real-world pose of the cell at (0,0) in the costmap |
| `resolutions` | `geometry_msgs/Vector3` | Resolution of the X, Y, and Z dimensions (meters per voxel) |
| `size_x` | `uint32` | Size of the voxel grid in the X dimension (number of voxels) |
| `size_y` | `uint32` | Size of the voxel grid in the Y dimension (number of voxels) |
| `size_z` | `uint32` | Size of the voxel grid in the Z dimension (number of voxels) |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.msg import VoxelGrid

class VoxelGridPublisher(Node):
    def __init__(self):
        super().__init__('voxelgrid_publisher')
        self.publisher = self.create_publisher(VoxelGrid, 'voxelgrid', 10)
        
    def publish_message(self):
        msg = VoxelGrid()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.data = []  # Fill array as needed
        # Set msg.origin as needed
        # Set msg.resolutions as needed
        msg.size_x = 0
        msg.size_y = 0
        msg.size_z = 0
        self.publisher.publish(msg)
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/msg/voxel_grid.hpp"

class VoxelGridPublisher : public rclcpp::Node
{
public:
    VoxelGridPublisher() : Node("voxelgrid_publisher")
    {
        publisher_ = create_publisher<nav2_msgs::msg::VoxelGrid>("voxelgrid", 10);
    }

    void publish_message()
    {
        auto msg = nav2_msgs::msg::VoxelGrid();
        msg.header.frame_id = "map";
        msg.header.stamp = this->now();
        // Fill msg.data array as needed
        // Set msg.origin as needed
        // Set msg.resolutions as needed
        msg.size_x = 0;
        msg.size_y = 0;
        msg.size_z = 0;
        publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<nav2_msgs::msg::VoxelGrid>::SharedPtr publisher_;
};
```

## Related Messages

- [All Costmap Messages](/humble/msgs/index.html#costmap-messages)
- [Message API Overview](/humble/msgs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
