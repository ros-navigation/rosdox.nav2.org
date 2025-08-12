---
layout: default
title: "TrajectoryPoint Message"
permalink: /msgs/humble/trajectorypoint.html
---

# TrajectoryPoint Message

**Package:** `nav2_msgs`  
**Category:** Navigation State Messages

Individual point in a trajectory with pose and velocity information

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `time_from_start` | `builtin_interfaces/Duration` | Time from trajectory start to reach this point |
| `pose` | `geometry_msgs/Pose` | Pose at this trajectory point |
| `velocity` | `geometry_msgs/Twist` | Velocity at this trajectory point |
| `acceleration` | `geometry_msgs/Accel` | Optional acceleration at this point |
| `effort` | `geometry_msgs/Wrench` | Optional force/torque to apply |

## Related Messages

- [Trajectory](/msgs/humble/trajectory.html) - Contains arrays of trajectory points
- [All Navigation State Messages](/msgs/humble/index.html#navigation-state-messages)

## Usage Context

Defines individual waypoints in planned robot trajectories with complete kinematic and dynamic information.

[Back to Message APIs](/msgs/humble/)
