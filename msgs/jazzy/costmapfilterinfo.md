---
layout: default
title: "CostmapFilterInfo Message"
permalink: /msgs/jazzy/costmapfilterinfo.html
---

# CostmapFilterInfo Message

**Package:** `nav2_msgs`  
**Category:** Costmap Messages

Filter information for costmap layers and processing

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `type` | `uint8` | Type of plugin used (keepout filter, speed limit in m/s, speed limit in percent, etc...) 0: keepout/lanes filter 1: speed limit filter in % of maximum speed 2: speed limit filter in absolute values (m/s) |
| `filter_mask_topic` | `string` | Name of filter mask topic |
| `base` | `float32` | Multiplier base offset and multiplier coefficient for conversion of OccGrid. Used to convert OccupancyGrid data values to filter space values. data -> into some other number space: space = data * multiplier + base |
| `multiplier` | `float32` | Floating point numerical value |



## Related Messages

- [All Costmap Messages](/jazzy/msgs/index.html#costmap-messages)
- [Message API Overview](/jazzy/msgs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
