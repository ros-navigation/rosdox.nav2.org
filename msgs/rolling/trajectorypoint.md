---
layout: default
title: "TrajectoryPoint Message"
permalink: /msgs/rolling/trajectorypoint.html
---

# TrajectoryPoint Message

**Package:** `nav2_msgs`  
**Category:** Planning Messages

Single point in a trajectory with pose, velocity, and timing

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `time_from_start` | `builtin_interfaces/Duration` | Trajectory point state Desired time from the trajectory start to arrive at this trajectory sample. |
| `pose` | `geometry_msgs/Pose` | Pose of the trajectory sample. |
| `velocity` | `geometry_msgs/Twist` | Velocity of the trajectory sample. |
| `acceleration` | `geometry_msgs/Accel` | Acceleration of the trajectory (optional). |
| `effort` | `geometry_msgs/Wrench` | Force/Torque to apply at trajectory sample (optional). |



## Related Messages

- [All Planning Messages](/rolling/msgs/index.html#planning-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
