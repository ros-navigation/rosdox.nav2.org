---
layout: default
title: "SpeedLimit Message"
permalink: /msgs/jazzy/speedlimit.html
---

# SpeedLimit Message

**Package:** `nav2_msgs`  
**Category:** Control Messages

Speed limit information for navigation areas

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `percentage` | `bool` | Setting speed limit in percentage if true or in absolute values in false case |
| `speed_limit` | `float64` | Maximum allowed speed (in percent of maximum robot speed or in m/s depending on "percentage" value). When no-limit it is set to 0.0 |



## Related Messages

- [All Control Messages](/jazzy/msgs/index.html#control-messages)
- [Message API Overview](/jazzy/msgs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
