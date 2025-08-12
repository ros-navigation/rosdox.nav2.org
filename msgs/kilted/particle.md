---
layout: default
title: "Particle Message"
permalink: /msgs/kilted/particle.html
---

# Particle Message

**Package:** `nav2_msgs`  
**Category:** Localization Messages

Individual particle representation for particle filter localization

## Message Definition

| Field | Type | Description |
|-------|------|-------------|
| `pose` | `geometry_msgs/Pose` | Spatial pose of the particle (position and orientation) |
| `weight` | `float64` | Weight/probability of the particle |

## Related Messages

- [ParticleCloud](/msgs/kilted/particlecloud.html) - Collection of particles for complete localization
- [All Localization Messages](/msgs/kilted/index.html#localization-messages)

## Usage Context

Particle messages are fundamental components of particle filter-based localization systems like AMCL (Adaptive Monte Carlo Localization). Each particle represents a hypothesis about the robot's pose in the environment.

### Particle Filter Localization
- **Initialization:** Particles are distributed across possible robot poses
- **Prediction:** Particles are moved based on odometry/motion model
- **Update:** Particle weights are updated based on sensor observations
- **Resampling:** High-weight particles are duplicated, low-weight particles are removed

### Weight Interpretation
- **Range:** Typically normalized so all particle weights sum to 1.0
- **High weights:** Particles that match sensor observations well
- **Low weights:** Particles that poorly explain sensor data
- **Zero weights:** Particles in impossible locations (e.g., inside walls)

### Navigation Integration
- Used by AMCL node for robot localization
- Particle distributions indicate localization confidence
- Narrow distributions suggest high confidence
- Wide distributions suggest uncertainty or kidnapped robot scenarios

### Debugging and Visualization
- Individual particles can be visualized in RViz
- Weight visualization helps understand localization quality
- Particle spread indicates localization uncertainty

[Back to Message APIs](/msgs/kilted/)
