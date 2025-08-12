---
layout: default
title: "Trajectory Message"
permalink: /msgs/kilted/trajectory.html
---

# Trajectory Message

**Package:** `nav2_msgs`  
**Category:** Navigation State Messages

Complete trajectory with sequence of trajectory points

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Indicates the frame_id of the trajectory |
| `points` | `TrajectoryPoint[]` | Array of trajectory points to follow |

## Related Messages

- [TrajectoryPoint](/msgs/kilted/trajectorypoint.html) - Individual points in the trajectory
- [All Navigation State Messages](/msgs/kilted/index.html#navigation-state-messages)

## Usage Context

Represents complete planned trajectories for robot motion, including spatial, temporal, and dynamic information.

[Back to Message APIs](/msgs/kilted/)
