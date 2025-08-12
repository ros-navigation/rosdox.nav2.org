---
layout: default
title: "Particle Message"
permalink: /msgs/jazzy/particle.html
---

# Particle Message

**Package:** `nav2_msgs`  
**Category:** Localization Messages

Single particle hypothesis with pose and weight for particle filter localization

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `pose` | `geometry_msgs/Pose` | This represents an individual particle with weight produced by a particle filter |
| `weight` | `float64` | Probability weight of this particle hypothesis |



## Related Messages

- [All Localization Messages](/jazzy/msgs/index.html#localization-messages)
- [Message API Overview](/jazzy/msgs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
