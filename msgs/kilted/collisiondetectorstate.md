---
layout: default
title: "CollisionDetectorState Message"
permalink: /msgs/kilted/collisiondetectorstate.html
---

# CollisionDetectorState Message

**Package:** `nav2_msgs`  
**Category:** Collision/Safety Messages

State information from collision detection systems

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `polygons` | `string[]` | Array of polygon names used for collision detection |
| `detections` | `bool[]` | Corresponding boolean array indicating detection status for each polygon |

## Related Messages

- [CollisionMonitorState](/msgs/kilted/collisionmonitorstate.html) - Monitoring data from collision avoidance systems
- [All Collision/Safety Messages](/msgs/kilted/index.html#collision-safety-messages)

## Usage Context

Provides real-time information about collision detection status across multiple detection zones, enabling safety monitoring and collision avoidance behaviors.

[Back to Message APIs](/msgs/kilted/)
