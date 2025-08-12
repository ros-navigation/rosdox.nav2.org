---
layout: default
title: "WaypointStatus Message"
permalink: /msgs/rolling/waypointstatus.html
---

# WaypointStatus Message

**Package:** `nav2_msgs`  
**Category:** Waypoint Messages

Status information for waypoint navigation tasks

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `waypoint_status` | `uint8` | Status of the waypoint (0=PENDING, 1=COMPLETED, 2=SKIPPED, 3=FAILED) |
| `waypoint_index` | `uint32` | Index of the current waypoint in sequence |
| `waypoint_pose` | `geometry_msgs/PoseStamped` | Pose of the waypoint |
| `error_code` | `uint16` | Numeric error code if operation failed |
| `error_msg` | `string` | Human-readable error message |



## Related Messages

- [All Waypoint Messages](/rolling/msgs/index.html#waypoint-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
