---
layout: default
title: "Route Message"
permalink: /msgs/jazzy/route.html
---

# Route Message

**Package:** `nav2_msgs`  
**Category:** Route/Planning Messages

Complete route definition with nodes and edges for graph-based navigation

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `route_cost` | `float32` | Total cost of the route |
| `nodes` | `RouteNode[]` | Ordered set of nodes in the route |
| `edges` | `RouteEdge[]` | Ordered set of edges connecting the nodes |

## Related Messages

- [RouteNode](/msgs/jazzy/routenode.html) - Individual nodes in the route
- [RouteEdge](/msgs/jazzy/routeedge.html) - Connections between route nodes
- [All Route/Planning Messages](/msgs/jazzy/index.html#route-planning-messages)

## Usage Context

Represents complete planned routes for graph-based navigation systems, containing both waypoints and connections.

[Back to Message APIs](/msgs/jazzy/)
