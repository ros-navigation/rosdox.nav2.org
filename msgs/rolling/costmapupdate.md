---
layout: default
title: "CostmapUpdate Message"
permalink: /msgs/rolling/costmapupdate.html
---

# CostmapUpdate Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

Update message containing changed regions of a costmap

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Update msg for Costmap containing the modified part of Costmap |
| `x` | `uint32` | X coordinate (grid cell index) of the lower-left corner of the update region |
| `y` | `uint32` | Y coordinate (grid cell index) of the lower-left corner of the update region |
| `size_x` | `uint32` | Width of the costmap window being updated (number of cells in X direction) |
| `size_y` | `uint32` | Height of the costmap window being updated (number of cells in Y direction) |
| `data` | `uint8[]` | The cost data, in row-major order, starting with (x,y) from 0-255 in Costmap format rather than OccupancyGrid 0-100. |



## Related Messages

- [All Costmap Messages](/rolling/msgs/index.html#costmap-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
