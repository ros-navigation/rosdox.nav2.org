---
layout: default
title: "EdgeCost Message"
permalink: /msgs/humble/edgecost.html
---

# EdgeCost Message

**Package:** `nav2_msgs`  
**Category:** Routing Messages

Cost information for route graph edges

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `edgeid` | `uint16` | Edge cost to use with nav2_msgs/srv/DynamicEdges to adjust route edge costs |
| `cost` | `float32` | Cost for traversing the edge |



## Related Messages

- [All Routing Messages](/humble/msgs/index.html#routing-messages)
- [Message API Overview](/humble/msgs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
