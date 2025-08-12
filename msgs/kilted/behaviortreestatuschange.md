---
layout: default
title: "BehaviorTreeStatusChange Message"
permalink: /msgs/kilted/behaviortreestatuschange.html
---

# BehaviorTreeStatusChange Message

**Package:** `nav2_msgs`  
**Category:** Behavior Tree Messages

Status change events from behavior tree node execution

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | `builtin_interfaces/Time` | Internal behavior tree event timestamp (wall clock time) |
| `node_name` | `string` | Name of the behavior tree node |
| `uid` | `uint16` | Unique identifier for this node |
| `previous_status` | `string` | Previous node status (IDLE, RUNNING, SUCCESS, or FAILURE) |
| `current_status` | `string` | Current node status (IDLE, RUNNING, SUCCESS, or FAILURE) |

## Related Messages

- [BehaviorTreeLog](/msgs/kilted/behaviortreelog.html) - Contains arrays of these status change events
- [All Behavior Tree Messages](/msgs/kilted/index.html#behavior-tree-messages)

## Usage Context

This message represents individual state transitions within Nav2's behavior tree execution. Each status change event provides detailed information about:

- Which specific node changed state
- What the transition was (from/to which status)
- When the transition occurred
- Unique identification of the node instance

These events are essential for:
- Real-time monitoring of behavior tree execution
- Debugging navigation behavior issues
- Performance profiling of different behavior tree nodes
- Creating detailed execution traces for analysis

The status transitions follow the standard behavior tree node lifecycle, where nodes transition between IDLE, RUNNING, SUCCESS, and FAILURE states based on their execution results.

[Back to Message APIs](/msgs/kilted/)
