---
layout: default
title: "ComputePathToPose Action"
permalink: /actions/rolling/computepathtopose.html
---

# ComputePathToPose Action

**Package:** `nav2_msgs`  
**Category:** Planning

Compute an optimal path from current position to a target pose

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `goal` | `geometry_msgs/PoseStamped` | Target goal pose for path planning |
| `start` | `geometry_msgs/PoseStamped` | Starting pose for path planning |
| `planner_id` | `string` | Name of the specific planning algorithm to use (e.g., "GridBased", "NavfnPlanner"). If empty with single planner, uses default |
| `use_start` | `bool` | If false, use current robot pose as path start, if true, use start above instead |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `NONE` | `uint16` | Success status code indicating the action completed without errors |
| `UNKNOWN` | `uint16` | Generic error code for unexpected or unclassified failures |
| `INVALID_PLANNER` | `uint16` | Error code indicating the specified planner plugin is invalid or not loaded |
| `TF_ERROR` | `uint16` | Error code indicating a transform/localization failure |
| `START_OUTSIDE_MAP` | `uint16` | Error code indicating the start position is outside the known map boundaries |
| `GOAL_OUTSIDE_MAP` | `uint16` | Error code indicating the goal position is outside the known map boundaries |
| `START_OCCUPIED` | `uint16` | Error code indicating the start position is in an occupied/blocked area |
| `GOAL_OCCUPIED` | `uint16` | Error code indicating the goal position is in an occupied/blocked area |
| `TIMEOUT` | `uint16` | Error code indicating the action exceeded its maximum allowed time |
| `NO_VALID_PATH` | `uint16` | Error code indicating no feasible path could be found between start and goal |
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `planning_time` | `builtin_interfaces/Duration` | Time spent in path planning phase |
| `error_code` | `uint16` | Numeric error code indicating specific failure reason (0=success, various codes for different failure types) |
| `error_msg` | `string` | Human-readable error message describing what went wrong during action execution |


### Feedback Message

No fields defined.


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathToPose

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, ComputePathToPose, 'compute_path_to_pose')
        
    def send_goal(self):
        goal_msg = ComputePathToPose.Goal()
        goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = 3.0
        goal_msg.goal.pose.position.y = 2.0
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.planner_id = 'GridBased'
        goal_msg.use_start = False
        
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
#include "nav2_msgs/action/compute_path_to_pose.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using ComputePathToPoseAction = nav2_msgs::action::ComputePathToPose;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathToPoseAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<ComputePathToPoseAction>(
            this, "compute_path_to_pose");
    }

    void send_goal()
    {
        auto goal_msg = ComputePathToPoseAction::Goal();
        goal_msg.goal.header.frame_id = "map";
        goal_msg.goal.pose.position.x = 3.0;
        goal_msg.goal.pose.position.y = 2.0;
        goal_msg.goal.pose.orientation.w = 1.0;
        goal_msg.planner_id = "GridBased";
        goal_msg.use_start = false;
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<ComputePathToPoseAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<ComputePathToPoseAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const ComputePathToPoseAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Planning Actions](/actions/rolling/index.html#planning)
- [Action API Overview](/actions/rolling/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
