---
layout: default
title: "ParticleCloud Message"
permalink: /msgs/kilted/particlecloud.html
---

# ParticleCloud Message

**Package:** `nav2_msgs`  
**Category:** Localization Messages

Collection of particles for particle filter-based localization

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `header` | `std_msgs/Header` | Standard ROS header with timestamp and frame information |
| `particles` | `Particle[]` | Array of particles representing the particle cloud |

## Related Messages

- [Particle](/msgs/kilted/particle.html) - Individual particles in the cloud
- [All Localization Messages](/msgs/kilted/index.html#localization-messages)

## Usage Context

Represents the complete set of pose hypotheses in particle filter localization systems like AMCL.

[Back to Message APIs](/msgs/kilted/)
