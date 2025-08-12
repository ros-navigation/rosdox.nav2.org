---
layout: default
title: "Route Message"
permalink: /msgs/humble/route.html
---

# Route Message

**Package:** `nav2_msgs`  
**Category:** Routing Messages

Complete route plan with waypoints and metadata

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `route_cost` | `float32` | Total cost for the route |
| `nodes` | `RouteNode[]` | ordered set of nodes of the route |
| `edges` | `RouteEdge[]` | ordered set of edges that connect nodes |



## Related Messages

- [All Routing Messages](/humble/msgs/index.html#routing-messages)
- [Message API Overview](/humble/msgs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
