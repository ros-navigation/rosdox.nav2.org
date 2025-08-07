---
layout: home
title: "ROS 2 API Docs"
---

# ROS 2 API Docs

Welcome to the C++ developer-friendly ROS 2 API documentation. This site provides comprehensive Doxygen-generated documentation for multiple ROS 2 distributions because I (Your Friendly Neighborhood Navigator) find the new format utterly unreadable and I needed a new solution before my total descent into madness.

## Available Distributions

### ROS 2 rclcpp + RCL API Documentation

<div class="distribution-grid">
  {% assign sorted_distributions = site.distributions | sort: 'order' %}
  {% for dist in sorted_distributions %}
  <div class="distribution-card">
    <h3><a href="/{{ dist.slug }}/html/index.html">{{ dist.title }}</a></h3>
    <p>{{ dist.description }}</p>
    <p><strong>Status:</strong> {{ dist.status }}</p>
  </div>
  {% endfor %}
</div>

### Nav2 API Documentation

<div class="distribution-grid">
  {% assign sorted_nav2_distributions = site.nav2_distributions | sort: 'order' %}
  {% for dist in sorted_nav2_distributions %}
  <div class="distribution-card nav2-card">
    <h3><a href="/nav2-{{ dist.slug }}/html/index.html">{{ dist.title }}</a></h3>
    <p>{{ dist.description }}</p>
    <p><strong>Status:</strong> {{ dist.status }}</p>
  </div>
  {% endfor %}
</div>

## About

This documentation is automatically generated weekly from the latest ROS 2 rclcpp, RCL, and Nav2 source code. The site provides:

- Complete API reference for rclcpp, RCL, and Nav2
- Cross-referenced source code with call graphs
- Enhanced cross-referencing between rclcpp APIs and RCL implementations
- Nav2 navigation stack comprehensive documentation
- Search functionality
- Mobile-friendly interface

## Resources

- [ROS 2 Official Documentation](https://docs.ros.org/)
- [rclcpp GitHub Repository](https://github.com/ros2/rclcpp)
- [RCL GitHub Repository](https://github.com/ros2/rcl)
- [Nav2 GitHub Repository](https://github.com/ros-navigation/navigation2)
- [Nav2 Project](https://nav2.org/)