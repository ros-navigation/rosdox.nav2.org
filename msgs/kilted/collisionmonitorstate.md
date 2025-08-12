---
layout: default
title: "CollisionMonitorState Message"
permalink: /msgs/kilted/collisionmonitorstate.html
---

# CollisionMonitorState Message

**Package:** `nav2_msgs`  
**Category:** Collision/Safety Messages

Monitoring data from collision avoidance systems

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `action_type` | `uint8` | Action to take (0=DO_NOTHING, 1=STOP, 2=SLOWDOWN, 3=APPROACH, 4=LIMIT) |
| `polygon_name` | `string` | Name of the polygon that triggered the action |

## Related Messages

- [CollisionDetectorState](/msgs/kilted/collisiondetectorstate.html) - State information from collision detection systems
- [All Collision/Safety Messages](/msgs/kilted/index.html#collision-safety-messages)

## Usage Context

Used by Nav2's collision monitor to communicate required safety actions based on detected obstacles in different zones around the robot.

[Back to Message APIs](/msgs/kilted/)
