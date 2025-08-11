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
| `smoother_id` | `string` | ID of Smoother Plugin to utilize|
| `max_smoothing_duration` | `builtin_interfaces/Duration` | The maximum time allowed to smooth|
| `check_for_collisions` | `bool` | Whether or not to check for collisions on the smoothed path|


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `smoothing_duration` | `builtin_interfaces/Duration` | Total time spent smoothing the path|
| `was_completed` | `bool` | If this smoothing was completed without collision or other issue|


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
        goal_msg.smoother_id = 'ConstrainedSmoother'
        goal_msg.max_smoothing_duration = Duration(seconds=5.0)
        goal_msg.check_for_collisions = True
        
        # Create a sample path to smooth
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        
        goal_msg.path = Path()
        goal_msg.path.header.frame_id = 'map'
        goal_msg.path.header.stamp = self.get_clock().now().to_msg()
        
        # Add some poses to the path
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(i)
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            goal_msg.path.poses.append(pose)
        
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
        goal_msg.smoother_id = "ConstrainedSmoother";
        goal_msg.max_smoothing_duration = rclcpp::Duration::from_seconds(5.0);
        goal_msg.check_for_collisions = true;
        
        // Create a sample path to smooth
        goal_msg.path.header.frame_id = "map";
        goal_msg.path.header.stamp = this->now();
        
        // Add some poses to the path
        for (int i = 0; i < 5; ++i) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = static_cast<double>(i);
            pose.pose.position.y = 0.0;
            pose.pose.orientation.w = 1.0;
            goal_msg.path.poses.push_back(pose);
        }
        
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

- [All Planning Actions](/humble/actions/index.html#planning)
- [Action API Overview](/humble/actions/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
