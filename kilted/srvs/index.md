---
layout: default
title: "Nav2 Service APIs - Kilted"
permalink: /kilted/srvs/
---

# Nav2 Service APIs - Kilted

This page documents the ROS Service APIs available in Nav2 for the kilted distribution. These services provide synchronous interfaces for various navigation operations including costmap management, route configuration, and map operations.

## Available Services (13 total)

### Costmap Services

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/srvs/kilted/clearcostmaparoundpose.html">ClearCostmapAroundPose</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Clear costmap within a specified distance around a given pose</p>
  </div>
  <div class="action-card">
    <h4><a href="/srvs/kilted/clearcostmaparoundrobot.html">ClearCostmapAroundRobot</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Clear costmap within a specified distance around the robot's current position</p>
  </div>
  <div class="action-card">
    <h4><a href="/srvs/kilted/clearcostmapexceptregion.html">ClearCostmapExceptRegion</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Clear entire costmap except for a specified rectangular region</p>
  </div>
  <div class="action-card">
    <h4><a href="/srvs/kilted/clearentirecostmap.html">ClearEntireCostmap</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Clear the entire costmap, resetting all cells to free space</p>
  </div>
  <div class="action-card">
    <h4><a href="/srvs/kilted/getcosts.html">GetCosts</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Retrieve costmap cost values at specified poses</p>
  </div>
  <div class="action-card">
    <h4><a href="/srvs/kilted/getcostmap.html">GetCostmap</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Retrieve the entire costmap as an occupancy grid</p>
  </div>
</div>

### Nav2 Route Services

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/srvs/kilted/setroutegraph.html">SetRouteGraph</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Load a new route graph from a specified file path</p>
  </div>
  <div class="action-card">
    <h4><a href="/srvs/kilted/dynamicedges.html">DynamicEdges</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Dynamically modify route graph edges during navigation</p>
  </div>
</div>

### Map Services

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/srvs/kilted/loadmap.html">LoadMap</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Load a map from a specified URL or file path</p>
  </div>
  <div class="action-card">
    <h4><a href="/srvs/kilted/savemap.html">SaveMap</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Save the current map to a specified file location</p>
  </div>
</div>

### Other Services

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/srvs/kilted/ispathvalid.html">IsPathValid</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Validate whether a given path is collision-free and traversable</p>
  </div>
  <div class="action-card">
    <h4><a href="/srvs/kilted/managelifecyclenodes.html">ManageLifecycleNodes</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Control lifecycle states of Nav2 nodes (startup, shutdown, pause, etc.)</p>
  </div>
  <div class="action-card">
    <h4><a href="/srvs/kilted/reloaddockdatabase.html">ReloadDockDatabase</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Reload the docking database with updated dock configurations</p>
  </div>
  <div class="action-card">
    <h4><a href="/srvs/kilted/setinitialpose.html">SetInitialPose</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Set the initial pose estimate for robot localization</p>
  </div>
</div>

## Related Documentation

- [Nav2 C++ API Documentation](/kilted/html/index.html)
- [Nav2 Action APIs](/kilted/actions/)
- [Nav2 Official Documentation](https://nav2.org/)
- [ROS 2 Services Documentation](https://docs.ros.org/en/kilted/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

*Generated on 2025-08-11 18:15:00 UTC*