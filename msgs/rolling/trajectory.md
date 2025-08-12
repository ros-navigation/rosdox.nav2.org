---
layout: default
title: "Trajectory Message"
permalink: /msgs/rolling/trajectory.html
---

# Trajectory Message

**Package:** `nav2_msgs`  
**Category:** Planning Messages

Time-parameterized path with poses and velocities

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | An array of trajectory points that represents a trajectory for a robot to follow. Indicates the frame_id of the trajectory. |
| `points` | `TrajectoryPoint[]` | Array of trajectory points to follow. |



## Related Messages

- [All Planning Messages](/rolling/msgs/index.html#planning-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
