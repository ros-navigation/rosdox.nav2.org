---
layout: default
title: "CostmapMetaData Message"
permalink: /msgs/rolling/costmapmetadata.html
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



## Related Messages

- [All Costmap Messages](/rolling/msgs/index.html#costmap-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
