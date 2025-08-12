---
layout: default
title: "CostmapUpdate Message"
permalink: /msgs/kilted/costmapupdate.html
---

# CostmapUpdate Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

Incremental updates to costmap data for efficient real-time updates

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `x` | `uint32` | X coordinate (grid cell index) of the lower-left corner of the update region |
| `y` | `uint32` | Y coordinate (grid cell index) of the lower-left corner of the update region |
| `size_x` | `uint32` | Width of the costmap window being updated (number of cells in X direction) |
| `size_y` | `uint32` | Height of the costmap window being updated (number of cells in Y direction) |
| `data` | `uint8[]` | Cost data (0-255) in row-major order starting from (x,y) |

## Related Messages

- [Costmap](/msgs/kilted/costmap.html) - Full costmap representation
- [All Costmap Messages](/msgs/kilted/index.html#costmap-messages)

## Usage Context

Enables efficient transmission of partial costmap changes, reducing bandwidth compared to full costmap republication.

[Back to Message APIs](/msgs/kilted/)
