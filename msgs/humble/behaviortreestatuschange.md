---
layout: default
title: "BehaviorTreeStatusChange Message"
permalink: /msgs/humble/behaviortreestatuschange.html
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
| `previous_status` | `string` | IDLE, RUNNING, SUCCESS or FAILURE |
| `current_status` | `string` | IDLE, RUNNING, SUCCESS or FAILURE |



## Related Messages

- [All Behavior Tree Messages](/humble/msgs/index.html#behavior-tree-messages)
- [Message API Overview](/humble/msgs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
