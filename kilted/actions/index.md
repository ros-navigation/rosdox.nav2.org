---
layout: default
title: "Nav2 Action APIs - Kilted"
permalink: /kilted/actions/
---

# Nav2 Action APIs - Kilted

This page documents the ROS Action APIs available in Nav2 for the kilted distribution. These actions provide high-level interfaces for navigation tasks, planning, and recovery behaviors.

## Available Actions (17 total)


### Navigation Actions

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/actions/kilted/followgpswaypoints.html">FollowGPSWaypoints</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Navigate robot through GPS-based waypoints for outdoor navigation</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/followwaypoints.html">FollowWaypoints</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Navigate robot through a series of waypoints with optional task execution at each point</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/navigatethroughposes.html">NavigateThroughPoses</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Navigate robot through a sequence of poses in order</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/navigatetopose.html">NavigateToPose</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Navigate robot to a specific pose with obstacle avoidance and recovery behaviors</p>
  </div>
</div>

### Planning Actions

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/actions/kilted/computeandtrackroute.html">ComputeAndTrackRoute</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Compute and actively track a route with dynamic replanning</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/computepaththroughposes.html">ComputePathThroughPoses</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Compute an optimal path connecting multiple poses in sequence</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/computepathtopose.html">ComputePathToPose</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Compute an optimal path from current position to a target pose</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/computeroute.html">ComputeRoute</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Compute a high-level route between waypoints using graph search</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/smoothpath.html">SmoothPath</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Generate a smoother, kinematically feasible path from a discrete path</p>
  </div>
</div>

### Controller Actions

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/actions/kilted/followpath.html">FollowPath</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Execute path following using a specified controller with progress monitoring</p>
  </div>
</div>

### Behaviors

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/actions/kilted/assistedteleop.html">AssistedTeleop</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Provide assisted teleoperation with collision avoidance and safety checks</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/backup.html">BackUp</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Move robot backwards a specified distance with obstacle detection</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/driveonheading.html">DriveOnHeading</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Drive robot forward in a specific direction for a given distance</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/spin.html">Spin</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Rotate robot in place to a target yaw angle with collision checking</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/wait.html">Wait</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Pause robot operation for a specified duration or until condition is met</p>
  </div>
</div>

### AutoDocking

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/actions/kilted/dockrobot.html">DockRobot</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Autonomously dock robot to a charging station or docking platform</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/kilted/undockrobot.html">UndockRobot</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Autonomously undock robot from a charging station or docking platform</p>
  </div>
</div>


## Related Documentation

- [Nav2 C++ API Documentation](/kilted/html/index.html)
- [Nav2 Official Documentation](https://nav2.org/)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/kilted/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

*Generated on 2025-08-08 09:48:43 UTC*