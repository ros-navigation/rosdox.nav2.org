---
layout: default
title: "EdgeCost Message"
permalink: /msgs/jazzy/edgecost.html
---

# EdgeCost Message

**Package:** `nav2_msgs`  
**Category:** Route/Planning Messages

Cost information for edges in route planning

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `edgeid` | `uint16` | Unique identifier for the edge |
| `cost` | `float32` | Cost for traversing the edge |

## Related Messages

- [RouteEdge](/msgs/jazzy/routeedge.html) - Edge definitions that use these costs
- [Route](/msgs/jazzy/route.html) - Routes that incorporate edge costs
- [All Route/Planning Messages](/msgs/jazzy/index.html#route-planning-messages)

## Usage Context

Used with nav2_msgs/srv/DynamicEdges service to dynamically adjust route edge costs during navigation.

[Back to Message APIs](/msgs/jazzy/)
