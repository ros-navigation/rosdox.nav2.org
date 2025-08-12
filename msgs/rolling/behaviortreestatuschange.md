---
layout: default
title: "BehaviorTreeStatusChange Message"
permalink: /msgs/rolling/behaviortreestatuschange.html
---

# BehaviorTreeStatusChange Message

**Package:** `nav2_msgs`  
**Category:** Behavior Tree Messages

Status change events from behavior tree nodes

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | `builtin_interfaces/Time` | internal behavior tree event timestamp. Typically this is wall clock time |
| `node_name` | `string` | String value or identifier |
| `uid` | `uint16` | unique ID for this node |
| `previous_status` | `string` | IDLE, RUNNING, SUCCESS or FAILURE |
| `current_status` | `string` | IDLE, RUNNING, SUCCESS or FAILURE |



## Related Messages

- [All Behavior Tree Messages](/rolling/msgs/index.html#behavior-tree-messages)
- [Message API Overview](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
