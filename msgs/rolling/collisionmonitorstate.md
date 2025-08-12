---
layout: default
title: "CollisionMonitorState Message"
permalink: /msgs/rolling/collisionmonitorstate.html
---

# CollisionMonitorState Message

**Package:** `nav2_msgs`  
**Category:** Collision Messages

Monitoring state for collision avoidance systems

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `DO_NOTHING` = `0 # No action` | `uint8` | Action type for robot in Collision Monitor. No action |
| `STOP` = `1 # Stop the robot` | `uint8` | Stop the robot |
| `SLOWDOWN` = `2 # Slowdown in percentage from current operating speed` | `uint8` | Slowdown in percentage from current operating speed |
| `APPROACH` = `3 # Keep constant time interval before collision` | `uint8` | Keep constant time interval before collision |
| `LIMIT` = `4 # Sets a limit of velocities if pts in range` | `uint8` | Sets a limit of velocities if pts in range |
| `action_type` | `uint8` | Message field - see Nav2 documentation for specific usage details |
| `polygon_name` | `string` | Name of triggered polygon |



## Related Messages

- [All Collision Messages](/rolling/msgs/index.html#collision-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
