---
layout: default
title: "BehaviorTreeLog Message"
permalink: /msgs/kilted/behaviortreelog.html
---

# BehaviorTreeLog Message

**Package:** `nav2_msgs`  
**Category:** Behavior Tree Messages

Log data from behavior tree execution including events and status changes

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `timestamp` | `builtin_interfaces/Time` | ROS time when the log message was sent |
| `event_log` | `BehaviorTreeStatusChange[]` | Array of behavior tree status change events |

## Related Messages

- [BehaviorTreeStatusChange](/msgs/kilted/behaviortreestatuschange.html) - Individual status change events contained in the log
- [All Behavior Tree Messages](/msgs/kilted/index.html#behavior-tree-messages)

## Usage Context

This message is typically published by Nav2's behavior tree executor to provide comprehensive logging of behavior tree execution. It allows external systems to monitor and analyze the execution flow of behavior trees used in navigation tasks.

The log contains a collection of status change events that occurred within a specific time window, making it useful for:
- Debugging behavior tree execution
- Performance analysis of navigation behaviors
- Historical analysis of robot decision-making
- Integration with external monitoring systems

[Back to Message APIs](/msgs/kilted/)
