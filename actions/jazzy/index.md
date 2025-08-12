---
layout: default
title: "Nav2 Action APIs - Jazzy"
permalink: /actions/jazzy/
---

# Nav2 Action APIs - Jazzy

This page documents the ROS Action APIs available in Nav2 for the jazzy distribution. These actions provide high-level interfaces for navigation tasks, planning, and recovery behaviors.

## Available Actions (15 total)


### Navigation Actions

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/actions/jazzy/followgpswaypoints.html">FollowGPSWaypoints</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Navigate robot through GPS-based waypoints for outdoor navigation</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/jazzy/followwaypoints.html">FollowWaypoints</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Navigate robot through a series of waypoints with optional task execution at each point</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/jazzy/navigatethroughposes.html">NavigateThroughPoses</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Navigate robot through a sequence of poses in order</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/jazzy/navigatetopose.html">NavigateToPose</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Navigate robot to a specific pose with obstacle avoidance and recovery behaviors</p>
  </div>
</div>

### Planning Actions

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/actions/jazzy/computepaththroughposes.html">ComputePathThroughPoses</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Compute an optimal path connecting multiple poses in sequence</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/jazzy/computepathtopose.html">ComputePathToPose</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Compute an optimal path from current position to a target pose</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/jazzy/smoothpath.html">SmoothPath</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Generate a smoother, kinematically feasible path from a discrete path</p>
  </div>
</div>

### Controller Actions

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/actions/jazzy/followpath.html">FollowPath</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Execute path following using a specified controller with progress monitoring</p>
  </div>
</div>

### Behaviors

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/actions/jazzy/assistedteleop.html">AssistedTeleop</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Provide assisted teleoperation with collision avoidance and safety checks</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/jazzy/backup.html">BackUp</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Move robot backwards a specified distance with obstacle detection</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/jazzy/driveonheading.html">DriveOnHeading</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Drive robot forward in a specific direction for a given distance</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/jazzy/spin.html">Spin</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Rotate robot in place to a target yaw angle with collision checking</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/jazzy/wait.html">Wait</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Pause robot operation for a specified duration or until condition is met</p>
  </div>
</div>

### AutoDocking

<div class="action-grid">
  <div class="action-card">
    <h4><a href="/actions/jazzy/dockrobot.html">DockRobot</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Autonomously dock robot to a charging station or docking platform</p>
  </div>
  <div class="action-card">
    <h4><a href="/actions/jazzy/undockrobot.html">UndockRobot</a></h4>
    <p class="action-package">Package: nav2_msgs</p>
    <p class="action-description">Autonomously undock robot from a charging station or docking platform</p>
  </div>
</div>


## Related Documentation

- [Nav2 C++ API Documentation](/jazzy/html/index.html)
- [Nav2 Official Documentation](https://nav2.org/)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/jazzy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

*Generated on 2025-08-12 08:36:22 UTC*