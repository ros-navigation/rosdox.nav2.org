---
layout: default
title: "RouteNode Message"
permalink: /msgs/kilted/routenode.html
---

# RouteNode Message

**Package:** `nav2_msgs`  
**Category:** Route/Planning Messages

Node in a route graph representing waypoints or locations

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `nodeid` | `uint16` | ID of the node in the graph |
| `position` | `geometry_msgs/Point` | Position of the node in space |

## Related Messages

- [Route](/msgs/kilted/route.html) - Contains arrays of route nodes
- [RouteEdge](/msgs/kilted/routeedge.html) - Connections between nodes
- [All Route/Planning Messages](/msgs/kilted/index.html#route-planning-messages)

## Usage Context

Represents individual waypoints or locations in graph-based navigation systems.

[Back to Message APIs](/msgs/kilted/)
