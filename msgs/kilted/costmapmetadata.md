---
layout: default
title: "CostmapMetaData Message"
permalink: /msgs/kilted/costmapmetadata.html
---

# CostmapMetaData Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

Metadata describing costmap properties and configuration

## Message Definition

| Field | Type | Description |
|-------|------|-----------|
| `map_load_time` | `builtin_interfaces/Time` | When the static map was loaded |
| `update_time` | `builtin_interfaces/Time` | Last update time to the costmap |
| `layer` | `string` | Name of the corresponding layer |
| `resolution` | `float32` | Map resolution in meters per cell |
| `size_x` | `uint32` | Number of cells horizontally |
| `size_y` | `uint32` | Number of cells vertically |
| `origin` | `geometry_msgs/Pose` | Real-world pose of cell (0,0) |

## Related Messages

- [Costmap](/msgs/kilted/costmap.html) - Uses this metadata structure
- [All Costmap Messages](/msgs/kilted/index.html#costmap-messages)

## Usage Context

Provides essential metadata for interpreting costmap data, including spatial reference, timing, and layer identification.

[Back to Message APIs](/msgs/kilted/)
