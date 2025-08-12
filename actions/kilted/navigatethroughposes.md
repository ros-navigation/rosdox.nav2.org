---
layout: default
title: "NavigateThroughPoses Action"
permalink: /actions/kilted/navigatethroughposes.html
---

# NavigateThroughPoses Action

**Package:** `nav2_msgs`  
**Category:** Navigation

Navigate robot through a sequence of poses in order

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `poses` | `nav_msgs/Goals` | Array of poses defining waypoints or path |
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
| `waypoint_statuses` | `WaypointStatus[]` | Array of status information for each waypoint including success/failure state and execution details |


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `current_pose` | `geometry_msgs/PoseStamped` | Current robot pose during navigation |
| `navigation_time` | `builtin_interfaces/Duration` | Total time elapsed since navigation started |
| `estimated_time_remaining` | `builtin_interfaces/Duration` | Estimated time remaining to reach the goal |
| `number_of_recoveries` | `int16` | Count of recovery behaviors executed during navigation to overcome obstacles or failures |
| `distance_remaining` | `float32` | Approximate distance remaining to the goal |
| `number_of_poses_remaining` | `int16` | Count of poses/waypoints remaining to be visited in the navigation sequence |
| `waypoint_statuses` | `WaypointStatus[]` | Array of status information for each waypoint including success/failure state and execution details |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateThroughPoses

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
        
    def send_goal(self):
        goal_msg = NavigateThroughPoses.Goal()
        goal_msg.poses = []
        pose1 = geometry_msgs.msg.PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 1.0
        pose1.pose.position.y = 1.0
        pose1.pose.orientation.w = 1.0
        goal_msg.poses.append(pose1)
        
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
#include "nav2_msgs/action/navigate_through_poses.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using NavigateThroughPosesAction = nav2_msgs::action::NavigateThroughPoses;
    using GoalHandle = rclcpp_action::ClientGoalHandle<NavigateThroughPosesAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<NavigateThroughPosesAction>(
            this, "navigate_through_poses");
    }

    void send_goal()
    {
        auto goal_msg = NavigateThroughPosesAction::Goal();
        geometry_msgs::msg::PoseStamped pose1;
        pose1.header.frame_id = "map";
        pose1.pose.position.x = 1.0;
        pose1.pose.position.y = 1.0;
        pose1.pose.orientation.w = 1.0;
        goal_msg.poses.push_back(pose1);
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<NavigateThroughPosesAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<NavigateThroughPosesAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const NavigateThroughPosesAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Navigation Actions](/actions/kilted/index.html#navigation)
- [Action API Overview](/actions/kilted/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
