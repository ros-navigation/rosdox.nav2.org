---
layout: default
title: "ComputePathThroughPoses Action"
permalink: /actions/jazzy/computepaththroughposes.html
---

# ComputePathThroughPoses Action

**Package:** `nav2_msgs`  
**Category:** Planning

Compute an optimal path connecting multiple poses in sequence

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `goals` | `geometry_msgs/PoseStamped[]` | Pose with header information (frame_id and timestamp) |
| `start` | `geometry_msgs/PoseStamped` | Starting pose for path planning |
| `planner_id` | `string` | Name of the planner plugin to use for path planning |
| `use_start` | `bool` | If false, use current robot pose as path start, if true, use start above instead |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `NONE` | `uint16` | result definition Error codes Note: The expected priority order of the errors should match the message order |
| `UNKNOWN` | `uint16` | Integer numeric value |
| `INVALID_PLANNER` | `uint16` | Integer numeric value |
| `TF_ERROR` | `uint16` | Integer numeric value |
| `START_OUTSIDE_MAP` | `uint16` | Integer numeric value |
| `GOAL_OUTSIDE_MAP` | `uint16` | Integer numeric value |
| `START_OCCUPIED` | `uint16` | Integer numeric value |
| `GOAL_OCCUPIED` | `uint16` | Integer numeric value |
| `TIMEOUT` | `uint16` | Integer numeric value |
| `NO_VALID_PATH` | `uint16` | Integer numeric value |
| `NO_VIAPOINTS_GIVEN` | `uint16` | Integer numeric value |
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `planning_time` | `builtin_interfaces/Duration` | Time spent in path planning phase |
| `error_code` | `uint16` | Error code indicating the result status |
| `error_msg` | `string` | Text string parameter |


### Feedback Message

No fields defined.


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputePathThroughPoses

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, ComputePathThroughPoses, 'compute_path_through_poses')
        
    def send_goal(self):
        goal_msg = ComputePathThroughPoses.Goal()
        # Set appropriate fields for ComputePathThroughPoses
        
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
#include "nav2_msgs/action/compute_path_through_poses.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using ComputePathThroughPosesAction = nav2_msgs::action::ComputePathThroughPoses;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputePathThroughPosesAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<ComputePathThroughPosesAction>(
            this, "compute_path_through_poses");
    }

    void send_goal()
    {
        auto goal_msg = ComputePathThroughPosesAction::Goal();
        // Set appropriate fields for ComputePathThroughPoses
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<ComputePathThroughPosesAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<ComputePathThroughPosesAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const ComputePathThroughPosesAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Planning Actions](/jazzy/actions/index.html#planning)
- [Action API Overview](/jazzy/actions/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
