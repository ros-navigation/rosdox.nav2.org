---
layout: default
title: "VoxelGrid Message"
permalink: /msgs/kilted/voxelgrid.html
---

# VoxelGrid Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

3D voxel grid representation for obstacle tracking and collision detection

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `data` | `uint32[]` | Voxel data array representing 3D occupancy information |
| `origin` | `geometry_msgs/Point32` | Origin point of the voxel grid in 3D space |
| `resolutions` | `geometry_msgs/Vector3` | Resolution of the X, Y, and Z dimensions (meters per voxel) |
| `size_x` | `uint32` | Number of voxels in X dimension |
| `size_y` | `uint32` | Number of voxels in Y dimension |
| `size_z` | `uint32` | Number of voxels in Z dimension |

## Related Messages

- [Costmap](/msgs/kilted/costmap.html) - 2D counterpart for navigation planning
- [All Costmap Messages](/msgs/kilted/index.html#costmap-messages)

## Usage Context

The VoxelGrid message is used in Nav2's 3D obstacle tracking and costmap generation, particularly in the voxel layer of costmap_2d.

### 3D Obstacle Tracking
- **Sensor Integration:** Combines data from 3D sensors (3D lidars, depth cameras)
- **Clearing and Marking:** Tracks which voxels are clear vs occupied
- **Temporal Persistence:** Maintains obstacle information over time

### Costmap Integration
- **Projection to 2D:** VoxelGrid data is projected onto 2D costmaps
- **Obstacle Layer:** Used by obstacle costmap layer for dynamic obstacles
- **Height Filtering:** Only obstacles within robot height range affect navigation

### Data Encoding
The uint32 data array typically uses bit flags:
- **Marked bits:** Indicate voxels with detected obstacles
- **Cleared bits:** Indicate voxels confirmed as free space
- **Unknown bits:** Indicate voxels with no sensor information

### Performance Considerations
- **Memory Usage:** 3D grids can be memory-intensive
- **Resolution Trade-offs:** Higher resolution improves accuracy but increases computation
- **Sensor Range:** Grid size should match sensor effective range

### Common Applications
- **Autonomous vehicles** with 3D lidar sensors
- **Mobile robots** operating in environments with overhangs
- **Drones** requiring full 3D obstacle avoidance
- **Industrial robots** in complex 3D environments

[Back to Message APIs](/msgs/kilted/)
