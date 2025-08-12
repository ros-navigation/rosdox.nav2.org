---
layout: default
title: "Costmap Message"
permalink: /msgs/humble/costmap.html
---

# Costmap Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

Occupancy grid representation for navigation planning with cost values

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | This represents a 2-D grid map, in which each cell has an associated cost |
| `metadata` | `CostmapMetaData` | MetaData for the map |
| `data` | `uint8[]` | The cost data, in row-major order, starting with (0,0). |



## Related Messages

- [All Costmap Messages](/humble/msgs/index.html#costmap-messages)
- [Message API Overview](/humble/msgs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
