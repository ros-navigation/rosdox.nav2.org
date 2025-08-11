---
layout: default
title: "SmoothPath Action"
permalink: /actions/rolling/smoothpath.html
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
| `smoother_id` | `string` | Text string parameter |
| `max_smoothing_duration` | `builtin_interfaces/Duration` | Time duration value |
| `check_for_collisions` | `bool` | Boolean true/false flag |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `smoothing_duration` | `builtin_interfaces/Duration` | Time duration value |
| `was_completed` | `bool` | Boolean true/false flag |
| `error_code` | `uint16` | Error code indicating the result status. Possible values: NONE, UNKNOWN, INVALID_SMOOTHER, TIMEOUT, SMOOTHED_PATH_IN_COLLISION, FAILED_TO_SMOOTH_PATH, INVALID_PATH|
| `error_msg` | `string` | Human readable error message that corresponds to the error code, when set|


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
        # Set appropriate fields for SmoothPath
        
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
        // Set appropriate fields for SmoothPath
        
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

- [All Planning Actions](/rolling/actions/index.html#planning)
- [Action API Overview](/rolling/actions/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
