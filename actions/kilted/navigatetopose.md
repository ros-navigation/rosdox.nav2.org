---
layout: default
title: "NavigateToPose Action"
permalink: /actions/kilted/navigatetopose.html
---

# NavigateToPose Action

**Package:** `nav2_msgs`  
**Category:** Navigation

Navigate robot to a specific pose with obstacle avoidance and recovery behaviors

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `pose` | `geometry_msgs/PoseStamped` | Target pose for navigation in the specified frame |
| `behavior_tree` | `string` | Path to custom behavior tree XML file to use for this navigation task. If empty, uses default navigation behavior tree with planning, following, and recovery behaviors |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `NONE` | `uint16` | Success status code indicating the action completed without errors |
| `UNKNOWN` | `uint16` | Generic error code for unexpected or unclassified failures |
| `FAILED_TO_LOAD_BEHAVIOR_TREE` | `uint16` | Error code indicating the specified behavior tree file could not be loaded |
| `TF_ERROR` | `uint16` | Error code indicating a transform/localization failure |
| `TIMEOUT` | `uint16` | Error code indicating the action exceeded its maximum allowed time |
| `error_code` | `uint16` | Numeric error code indicating specific failure reason (0=success, various codes for different failure types) |
| `error_msg` | `string` | Human-readable error message describing what went wrong during action execution |


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `current_pose` | `geometry_msgs/PoseStamped` | Current robot pose during navigation |
| `navigation_time` | `builtin_interfaces/Duration` | Total time elapsed since navigation started |
| `estimated_time_remaining` | `builtin_interfaces/Duration` | Estimated time remaining to reach the goal |
| `number_of_recoveries` | `int16` | Count of recovery behaviors executed during navigation to overcome obstacles or failures |
| `distance_remaining` | `float32` | Approximate distance remaining to the goal |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 2.0
        goal_msg.pose.pose.position.y = 1.0
        goal_msg.pose.pose.orientation.w = 1.0
        
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
#include "nav2_msgs/action/navigate_to_pose.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateToPoseAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<NavigateToPoseAction>(
            this, "navigate_to_pose");
    }

    void send_goal()
    {
        auto goal_msg = NavigateToPoseAction::Goal();
        goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = 2.0;
        goal_msg.pose.pose.position.y = 1.0;
        goal_msg.pose.pose.orientation.w = 1.0;
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<NavigateToPoseAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateToPoseAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Navigation Actions](/actions/kilted/index.html#navigation)
- [Action API Overview](/actions/kilted/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
