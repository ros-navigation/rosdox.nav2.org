---
layout: default
title: "Costmap Message"
permalink: /msgs/kilted/costmap.html
---

# Costmap Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

2D grid map representation with cost values for navigation planning

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `metadata` | `CostmapMetaData` | Metadata about the costmap structure and properties |
| `data` | `uint8[]` | Cost data in row-major order, starting with (0,0) |

## Related Messages

- [CostmapMetaData](/msgs/kilted/costmapmetadata.html) - Metadata structure used in this message
- [CostmapUpdate](/msgs/kilted/costmapupdate.html) - Incremental updates to costmap data
- [All Costmap Messages](/msgs/kilted/index.html#costmap-messages)

## Usage Context

The Costmap message is fundamental to Nav2's navigation system, representing the robot's understanding of the environment for path planning and obstacle avoidance. It's used by:

### Planners
- **Global planners** use costmaps to find optimal paths from start to goal
- **Local planners** use costmaps for real-time obstacle avoidance

### Costmap Layers
Different costmap layers contribute to the final cost values:
- **Static layer** - Permanent obstacles from map data
- **Obstacle layer** - Dynamic obstacles from sensor data
- **Inflation layer** - Inflated costs around obstacles
- **Speed filter layer** - Speed-based cost modifications

### Navigation Stack Integration
- Published by costmap_2d nodes (global_costmap, local_costmap)
- Consumed by planners, controllers, and behaviors
- Used for visualization in RViz and other tools

### Performance Considerations
- Large costmaps can be bandwidth-intensive
- Consider using [CostmapUpdate](/msgs/kilted/costmapupdate.html) for incremental changes
- Resolution affects both accuracy and computational cost

[Back to Message APIs](/msgs/kilted/)
