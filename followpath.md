---
layout: default
title: "FollowPath Action"
permalink: /actions/rolling/followpath.html
---

# FollowPath Action

**Package:** `nav2_msgs`  
**Category:** Controller

Execute path following using a specified controller with progress monitoring

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `controller_id` | `string` | Name of the path following controller to use (e.g., "FollowPath", "RegulatedPurePursuit") |
| `goal_checker_id` | `string` | Name of the goal checker plugin to use |
| `progress_checker_id` | `string` | Name of the progress monitoring plugin to use for tracking path following advancement |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `NONE` | `uint16` | Success status code indicating the action completed without errors |
| `UNKNOWN` | `uint16` | Generic error code for unexpected or unclassified failures |
| `INVALID_CONTROLLER` | `uint16` | Error code indicating the specified controller plugin is invalid or not loaded |
| `TF_ERROR` | `uint16` | Error code indicating a transform/localization failure |
| `INVALID_PATH` | `uint16` | Error code indicating the provided path is malformed or contains invalid data |
| `PATIENCE_EXCEEDED` | `uint16` | Error code indicating the controller exceeded its patience limit waiting for progress |
| `FAILED_TO_MAKE_PROGRESS` | `uint16` | Error code indicating the robot failed to make sufficient progress along the path |
| `NO_VALID_CONTROL` | `uint16` | Error code indicating the controller could not compute valid control commands |
| `CONTROLLER_TIMED_OUT` | `uint16` | Error code indicating the controller exceeded its maximum allowed execution time |
| `result` | `std_msgs/Empty` | Empty result indicating successful completion |
| `error_code` | `uint16` | Numeric error code indicating specific failure reason (0=success, various codes for different failure types) |
| `error_msg` | `string` | Human-readable error message describing what went wrong during action execution |


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `distance_to_goal` | `float32` | Current distance from the robot to the final goal position in meters, updated continuously during navigation |
| `speed` | `float32` | Movement speed in meters per second for the specified motion |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowPath

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, FollowPath, 'follow_path')
        
    def send_goal(self):
        goal_msg = FollowPath.Goal()
        goal_msg.path.header.frame_id = 'map'
        goal_msg.path.header.stamp = self.get_clock().now().to_msg()
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'simple_goal_checker' 
        
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
#include "nav2_msgs/action/follow_path.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using FollowPathAction = nav2_msgs::action::FollowPath;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowPathAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<FollowPathAction>(
            this, "follow_path");
    }

    void send_goal()
    {
        auto goal_msg = FollowPathAction::Goal();
        goal_msg.path.header.frame_id = "map";
        goal_msg.path.header.stamp = this->now();
        goal_msg.controller_id = "FollowPath";
        goal_msg.goal_checker_id = "simple_goal_checker";
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<FollowPathAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<FollowPathAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const FollowPathAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Controller Actions](/actions/rolling/index.html#controller)
- [Action API Overview](/actions/rolling/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
