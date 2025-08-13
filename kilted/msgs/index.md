---
layout: default
title: "Nav2 Message APIs - Kilted"
permalink: /kilted/msgs/
---

# Nav2 Message APIs - Kilted

This page documents the ROS Message APIs available in Nav2 for the kilted distribution. These messages provide data structures for navigation tasks, sensor data, and system status.

## Available Messages (19 total)


### Costmap Messages

<div class="message-grid">
  <div class="message-card">
    <h4><a href="/msgs/kilted/costmap.html">Costmap</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Occupancy grid representation for navigation planning with cost values</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/costmapfilterinfo.html">CostmapFilterInfo</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Filter information for costmap layers and processing</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/costmapmetadata.html">CostmapMetaData</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Metadata information for costmap including dimensions and resolution</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/costmapupdate.html">CostmapUpdate</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Update message containing changed regions of a costmap</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/voxelgrid.html">VoxelGrid</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">3D voxel grid representation for obstacle detection and mapping</p>
  </div>
</div>

### Behavior Tree Messages

<div class="message-grid">
  <div class="message-card">
    <h4><a href="/msgs/kilted/behaviortreelog.html">BehaviorTreeLog</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Logging information from behavior tree execution</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/behaviortreestatuschange.html">BehaviorTreeStatusChange</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Status change events from behavior tree nodes</p>
  </div>
</div>

### Localization Messages

<div class="message-grid">
  <div class="message-card">
    <h4><a href="/msgs/kilted/particle.html">Particle</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Single particle hypothesis with pose and weight for particle filter localization</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/particlecloud.html">ParticleCloud</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Collection of particles representing pose distribution in particle filter</p>
  </div>
</div>

### Route Planning Messages

<div class="message-grid">
  <div class="message-card">
    <h4><a href="/msgs/kilted/edgecost.html">EdgeCost</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Cost information for route graph edges</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/route.html">Route</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Complete route plan with waypoints and metadata</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/routeedge.html">RouteEdge</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Connection between route nodes with traversal cost</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/routenode.html">RouteNode</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Individual waypoint node in a route graph</p>
  </div>
</div>

### Collision Detection Messages

<div class="message-grid">
  <div class="message-card">
    <h4><a href="/msgs/kilted/collisiondetectorstate.html">CollisionDetectorState</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">State information from collision detection systems</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/collisionmonitorstate.html">CollisionMonitorState</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Monitoring state for collision avoidance systems</p>
  </div>
</div>

### Path Planning Messages

<div class="message-grid">
  <div class="message-card">
    <h4><a href="/msgs/kilted/trajectory.html">Trajectory</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Time-parameterized path with poses and velocities</p>
  </div>
  <div class="message-card">
    <h4><a href="/msgs/kilted/trajectorypoint.html">TrajectoryPoint</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Single point in a trajectory with pose, velocity, and timing</p>
  </div>
</div>

### Control Messages

<div class="message-grid">
  <div class="message-card">
    <h4><a href="/msgs/kilted/speedlimit.html">SpeedLimit</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Speed limit information for navigation areas</p>
  </div>
</div>

### Waypoint Messages

<div class="message-grid">
  <div class="message-card">
    <h4><a href="/msgs/kilted/waypointstatus.html">WaypointStatus</a></h4>
    <p class="message-package">Package: nav2_msgs</p>
    <p class="message-description">Status information for waypoint navigation tasks</p>
  </div>
</div>


## Related Documentation

- [Nav2 Action APIs](/kilted/actions/index.html)
- [Nav2 Service APIs](/kilted/srvs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
- [Nav2 Official Documentation](https://nav2.org/)

*Generated on 2025-08-12 14:56:39 UTC*