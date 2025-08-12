---
layout: default
title: "SmoothPath Action"
permalink: /actions/humble/smoothpath.html
---

# SmoothPath Action

**Package:** `nav2_msgs`  
**Category:** Planning

Generate a smoother, kinematically feasible path from a discrete path

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `smoother_id` | `string` | Name of the path smoothing algorithm plugin to use |
| `max_smoothing_duration` | `builtin_interfaces/Duration` | Maximum time allowed for the path smoothing algorithm to compute and refine the path |
| `check_for_collisions` | `bool` | Whether to perform collision checking on the smoothed path to ensure safety |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `smoothing_duration` | `builtin_interfaces/Duration` | Actual time taken by the smoothing algorithm to process and optimize the path in seconds |
| `was_completed` | `bool` | Whether the smoothing operation finished successfully within the allocated time and computational limits |


### Feedback Message

No fields defined.


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import SmoothPath

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, SmoothPath, 'smooth_path')
        
    def send_goal(self):
        goal_msg = SmoothPath.Goal()
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        
        # Create path to smooth
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Add waypoints to path
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(i)
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        goal_msg.path = path
        goal_msg.smoother_id = 'simple_smoother'
        goal_msg.max_smoothing_duration = Duration(seconds=5.0)
        
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        return future
        
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {feedback_msg.feedback}')
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "nav2_msgs/action/smooth_path.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using SmoothPathAction = nav2_msgs::action::SmoothPath;
    using GoalHandle = rclcpp_action::ClientGoalHandle<SmoothPathAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<SmoothPathAction>(
            this, "smooth_path");
    }

    void send_goal()
    {
        auto goal_msg = SmoothPathAction::Goal();
        // Create path to smooth
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        
        // Add waypoints to path
        for (int i = 0; i < 5; i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = static_cast<double>(i);
            pose.pose.position.y = 0.0;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        
        goal_msg.path = path;
        goal_msg.smoother_id = "simple_smoother";
        goal_msg.max_smoothing_duration = rclcpp::Duration::from_seconds(5.0);
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<SmoothPathAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<SmoothPathAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const SmoothPathAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Planning Actions](/actions/humble/index.html#planning)
- [Action API Overview](/actions/humble/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
