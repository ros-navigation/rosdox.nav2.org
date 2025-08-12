---
layout: default
title: "CostmapFilterInfo Message"
permalink: /msgs/kilted/costmapfilterinfo.html
---

# CostmapFilterInfo Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

Information about costmap filters and their parameters

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `type` | `uint8` | Filter type (0=keepout/lanes, 1=speed limit %, 2=speed limit m/s) |
| `filter_mask_topic` | `string` | Topic name for the filter mask |
| `base` | `float32` | Base offset for OccupancyGrid conversion |
| `multiplier` | `float32` | Multiplier coefficient for OccupancyGrid conversion |

## Related Messages

- [All Costmap Messages](/msgs/kilted/index.html#costmap-messages)

## Usage Context

Provides configuration information for costmap filter layers, enabling dynamic interpretation of filter mask data for different types of navigation constraints.

[Back to Message APIs](/msgs/kilted/)
