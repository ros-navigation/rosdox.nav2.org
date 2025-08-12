---
layout: default
title: "WaypointStatus Message"
permalink: /msgs/jazzy/waypointstatus.html
---

# WaypointStatus Message

**Package:** `nav2_msgs`  
**Category:** Navigation State Messages

Status information for waypoint navigation tasks

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `waypoint_status` | `uint8` | Status of the waypoint (0=PENDING, 1=COMPLETED, 2=SKIPPED, 3=FAILED) |
| `waypoint_index` | `uint32` | Index of the waypoint in the sequence |
| `waypoint_pose` | `geometry_msgs/PoseStamped` | Pose of the waypoint with timestamp |
| `error_code` | `uint16` | Error code if waypoint failed |
| `error_msg` | `string` | Human-readable error message |

## Related Messages

- [All Navigation State Messages](/msgs/jazzy/index.html#navigation-state-messages)

## Usage Context

Provides status updates during waypoint navigation tasks, enabling monitoring and error handling for sequential navigation missions.

[Back to Message APIs](/msgs/jazzy/)
