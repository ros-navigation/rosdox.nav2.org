---
layout: default
title: "RouteEdge Message"
permalink: /msgs/jazzy/routeedge.html
---

# RouteEdge Message

**Package:** `nav2_msgs`  
**Category:** Route/Planning Messages

Edge connection between route nodes with cost information

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `edgeid` | `uint16` | ID of the edge in the graph |
| `start` | `geometry_msgs/Point` | Start point that defines the edge |
| `end` | `geometry_msgs/Point` | End point that defines the edge |

## Related Messages

- [Route](/msgs/jazzy/route.html) - Contains arrays of route edges
- [RouteNode](/msgs/jazzy/routenode.html) - Nodes connected by edges
- [EdgeCost](/msgs/jazzy/edgecost.html) - Cost information for edges
- [All Route/Planning Messages](/msgs/jazzy/index.html#route-planning-messages)

## Usage Context

Represents connections between waypoints in graph-based navigation, defining traversable paths.

[Back to Message APIs](/msgs/jazzy/)
