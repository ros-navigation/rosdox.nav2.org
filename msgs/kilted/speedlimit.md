---
layout: default
title: "SpeedLimit Message"
permalink: /msgs/kilted/speedlimit.html
---

# SpeedLimit Message

**Package:** `nav2_msgs`  
**Category:** Navigation State Messages

Speed limit constraints for navigation planning

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `percentage` | `bool` | Whether speed limit is in percentage (true) or absolute values (false) |
| `speed_limit` | `float64` | Maximum allowed speed (% or m/s based on percentage field) |

## Related Messages

- [All Navigation State Messages](/msgs/kilted/index.html#navigation-state-messages)

## Usage Context

Used by speed filter layers and navigation components to enforce dynamic speed constraints based on environmental conditions or safety requirements.

[Back to Message APIs](/msgs/kilted/)
