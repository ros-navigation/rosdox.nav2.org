---
layout: default
title: "VoxelGrid Message"
permalink: /msgs/jazzy/voxelgrid.html
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



## Related Messages

- [All Costmap Messages](/jazzy/msgs/index.html#costmap-messages)
- [Message API Overview](/jazzy/msgs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
