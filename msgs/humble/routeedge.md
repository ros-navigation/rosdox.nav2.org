---
layout: default
title: "RouteEdge Message"
permalink: /msgs/humble/routeedge.html
---

# RouteEdge Message

**Package:** `nav2_msgs`  
**Category:** Routing Messages

Connection between route nodes with traversal cost

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `edgeid` | `uint16` | ID of the edge in the graph |
| `start` | `geometry_msgs/Point` | Start point that defines the edge |
| `end` | `geometry_msgs/Point` | End point that defines the edge |



## Related Messages

- [All Routing Messages](/humble/msgs/index.html#routing-messages)
- [Message API Overview](/humble/msgs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
