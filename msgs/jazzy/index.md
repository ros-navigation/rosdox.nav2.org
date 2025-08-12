---
layout: default
title: "Nav2 Message APIs - Jazzy"
permalink: /msgs/jazzy/
---

# Nav2 Message APIs - Jazzy

This page documents the ROS Message APIs available in Nav2 for the jazzy distribution. These messages provide data structures for navigation tasks, sensor data, and system status.

## Available Messages (13 total)


### Costmap Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/jazzy/costmap.html">Costmap</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Occupancy grid representation for navigation planning with cost values</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/jazzy/costmapfilterinfo.html">CostmapFilterInfo</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Filter information for costmap layers and processing</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/jazzy/costmapmetadata.html">CostmapMetaData</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Metadata information for costmap including dimensions and resolution</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/jazzy/costmapupdate.html">CostmapUpdate</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Update message containing changed regions of a costmap</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/jazzy/voxelgrid.html">VoxelGrid</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">3D voxel grid representation for obstacle detection and mapping</p>
  </div>
</div>

### Behavior Tree Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/jazzy/behaviortreelog.html">BehaviorTreeLog</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Logging information from behavior tree execution</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/jazzy/behaviortreestatuschange.html">BehaviorTreeStatusChange</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Status change events from behavior tree nodes</p>
  </div>
</div>

### Localization Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/jazzy/particle.html">Particle</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Single particle hypothesis with pose and weight for particle filter localization</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/jazzy/particlecloud.html">ParticleCloud</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Collection of particles representing pose distribution in particle filter</p>
  </div>
</div>

### Collision Detection Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/jazzy/collisiondetectorstate.html">CollisionDetectorState</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">State information from collision detection systems</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/jazzy/collisionmonitorstate.html">CollisionMonitorState</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Monitoring state for collision avoidance systems</p>
  </div>
</div>

### Trajectory Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/jazzy/speedlimit.html">SpeedLimit</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Speed limit information for navigation areas</p>
  </div>
</div>

### Waypoint Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/jazzy/missedwaypoint.html">MissedWaypoint</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Nav2 message for robotic navigation and behavior control</p>
  </div>
</div>


## Related Documentation

- [Nav2 Action APIs](/actions/jazzy/index.html)
- [Nav2 Service APIs](/srvs/jazzy/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
- [Nav2 Official Documentation](https://nav2.org/)

*Generated on 2025-08-12 08:24:23 UTC*