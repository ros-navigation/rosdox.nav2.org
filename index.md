---
layout: home
title: "ROS 2 API Docs"
---

# ROS 2 API Docs

Welcome to the C++ developer-friendly ROS 2 API documentation. This site provides comprehensive Doxygen-generated documentation for multiple ROS 2 distributions because I (Your Friendly Neighborhood Navigator) find the new format utterly unreadable and I needed a new solution before my total descent into madness.

## Available Distributions

<div class="distribution-grid">
  {% assign sorted_distributions = site.distributions | sort: 'order' %}
  {% for dist in sorted_distributions %}
  <div class="distribution-card">
    <h3><a href="{{ dist.url }}">{{ dist.title }}</a></h3>
    <p>{{ dist.description }}</p>
    <p><strong>Status:</strong> {{ dist.status }}</p>
    <p><strong>Last Updated:</strong> {{ dist.last_updated | date: "%B %d, %Y" }}</p>
  </div>
  {% endfor %}
</div>

## About

This documentation is automatically generated weekly from the latest ROS 2 rclcpp source code. The site provides:

- Complete API reference for rclcpp
- Cross-referenced source code
- Search functionality
- Mobile-friendly interface

## Resources

- [ROS 2 Official Documentation](https://docs.ros.org/)
- [rclcpp GitHub Repository](https://github.com/ros2/rclcpp)
- [Nav2 Project](https://nav2.org/)