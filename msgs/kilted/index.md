---
layout: default
title: "Nav2 Message APIs - Kilted"
permalink: /msgs/kilted/
---

# Nav2 Message APIs - Kilted

This page documents the ROS Message APIs available in Nav2 for the kilted distribution. These messages define the data structures used for communication between navigation components, including behavior trees, costmaps, localization, and planning.

## Available Messages (19 total)

### Behavior Tree Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/kilted/behaviortreelog.html">BehaviorTreeLog</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Log data from behavior tree execution including events and status changes</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/behaviortreestatuschange.html">BehaviorTreeStatusChange</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Status change events from behavior tree node execution</p>
  </div>
</div>

### Costmap Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/kilted/costmap.html">Costmap</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">2D grid map representation with cost values for navigation planning</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/costmapfilterinfo.html">CostmapFilterInfo</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Information about costmap filters and their parameters</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/costmapmetadata.html">CostmapMetaData</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Metadata describing costmap properties and configuration</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/costmapupdate.html">CostmapUpdate</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Incremental updates to costmap data for efficient real-time updates</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/voxelgrid.html">VoxelGrid</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">3D voxel grid representation for obstacle tracking and collision detection</p>
  </div>
</div>

### Collision/Safety Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/kilted/collisiondetectorstate.html">CollisionDetectorState</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">State information from collision detection systems</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/collisionmonitorstate.html">CollisionMonitorState</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Monitoring data from collision avoidance systems</p>
  </div>
</div>

### Localization Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/kilted/particle.html">Particle</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Individual particle representation for particle filter localization</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/particlecloud.html">ParticleCloud</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Collection of particles for particle filter-based localization</p>
  </div>
</div>

### Route/Planning Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/kilted/edgecost.html">EdgeCost</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Cost information for edges in route planning</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/route.html">Route</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Complete route definition with nodes and edges for graph-based navigation</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/routeedge.html">RouteEdge</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Edge connection between route nodes with cost information</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/routenode.html">RouteNode</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Node in a route graph representing waypoints or locations</p>
  </div>
</div>

### Trajectory Messages

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/msgs/kilted/speedlimit.html">SpeedLimit</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Speed limit constraints for navigation planning</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/trajectory.html">Trajectory</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Complete trajectory with sequence of trajectory points</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/trajectorypoint.html">TrajectoryPoint</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Individual point in a trajectory with pose and velocity information</p>
  </div>
  <div class="action-card">
    <h4><a href="/msgs/kilted/waypointstatus.html">WaypointStatus</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Status information for waypoint navigation tasks</p>
  </div>
</div>


## Related Documentation

- [Nav2 C++ API Documentation](/kilted/html/index.html)
- [Nav2 Official Documentation](https://nav2.org/)
- [ROS 2 Messages Documentation](https://docs.ros.org/en/kilted/Concepts/About-ROS-Interfaces.html)

[Back to Home](/)