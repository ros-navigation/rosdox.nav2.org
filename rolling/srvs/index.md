---
layout: default
title: "Nav2 Service APIs - Rolling"
permalink: /rolling/srvs/
---

# Nav2 Service APIs - Rolling

This page documents the ROS Service APIs available in Nav2 for the rolling distribution. These services provide request-response interfaces for navigation configuration, system control, and data queries.

## Available Services (14 total)


### Costmap Services

<div class="service-grid">
  <div class="service-card">
    <h4><a href="/srvs/rolling/clearcostmaparoundpose.html">ClearCostmapAroundPose</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Clear costmap within a specified distance around a given pose</p>
  </div>
  <div class="service-card">
    <h4><a href="/srvs/rolling/clearcostmaparoundrobot.html">ClearCostmapAroundRobot</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Clear costmap within a specified distance around the robot's current position</p>
  </div>
  <div class="service-card">
    <h4><a href="/srvs/rolling/clearcostmapexceptregion.html">ClearCostmapExceptRegion</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Clear entire costmap except for a specified rectangular region</p>
  </div>
  <div class="service-card">
    <h4><a href="/srvs/rolling/clearentirecostmap.html">ClearEntireCostmap</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Clear the entire costmap, resetting all cells to free space</p>
  </div>
  <div class="service-card">
    <h4><a href="/srvs/rolling/getcostmap.html">GetCostmap</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Retrieve the entire costmap as an occupancy grid</p>
  </div>
  <div class="service-card">
    <h4><a href="/srvs/rolling/getcosts.html">GetCosts</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Retrieve costmap cost values at specified poses</p>
  </div>
</div>

### Route Planning Services

<div class="service-grid">
  <div class="service-card">
    <h4><a href="/srvs/rolling/dynamicedges.html">DynamicEdges</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Dynamically modify route graph edges during navigation</p>
  </div>
  <div class="service-card">
    <h4><a href="/srvs/rolling/setroutegraph.html">SetRouteGraph</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Load a new route graph from a specified file path</p>
  </div>
</div>

### Map Services

<div class="service-grid">
  <div class="service-card">
    <h4><a href="/srvs/rolling/loadmap.html">LoadMap</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Load a map from a specified URL or file path</p>
  </div>
  <div class="service-card">
    <h4><a href="/srvs/rolling/savemap.html">SaveMap</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Save the current map to a specified file location</p>
  </div>
</div>

### Docking Services

<div class="service-grid">
  <div class="service-card">
    <h4><a href="/srvs/rolling/reloaddockdatabase.html">ReloadDockDatabase</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Reload the docking database with updated dock configurations</p>
  </div>
</div>

### Lifecycle Services

<div class="service-grid">
  <div class="service-card">
    <h4><a href="/srvs/rolling/managelifecyclenodes.html">ManageLifecycleNodes</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Control lifecycle states of Nav2 nodes (startup, shutdown, pause, etc.)</p>
  </div>
</div>

### Validation Services

<div class="service-grid">
  <div class="service-card">
    <h4><a href="/srvs/rolling/ispathvalid.html">IsPathValid</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Validate whether a given path is collision-free and traversable</p>
  </div>
</div>

### Localization Services

<div class="service-grid">
  <div class="service-card">
    <h4><a href="/srvs/rolling/setinitialpose.html">SetInitialPose</a></h4>
    <p class="service-package">Package: nav2_msgs</p>
    <p class="service-description">Set the initial pose estimate for robot localization</p>
  </div>
</div>


## Related Documentation

- [Nav2 Action APIs](/rolling/actions/index.html)
- [Nav2 Message APIs](/rolling/msgs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
- [Nav2 Official Documentation](https://nav2.org/)

*Generated on 2025-08-12 14:56:47 UTC*