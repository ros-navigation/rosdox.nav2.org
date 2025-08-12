---
layout: default
title: "MissedWaypoint Message"
permalink: /msgs/jazzy/missedwaypoint.html
---

# MissedWaypoint Message

**Package:** `nav2_msgs`  
**Category:** Waypoint Messages

Nav2 message for robotic navigation and behavior control

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `index` | `uint32` | Integer numerical value |
| `goal` | `geometry_msgs/PoseStamped` | Pose with header information (frame_id and timestamp) |
| `error_code` | `uint16` | Numeric error code if operation failed |



## Related Messages

- [All Waypoint Messages](/jazzy/msgs/index.html#waypoint-messages)
- [Message API Overview](/jazzy/msgs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
