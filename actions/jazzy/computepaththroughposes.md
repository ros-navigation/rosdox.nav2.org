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
| `goals` | `geometry_msgs/PoseStamped[]` | Vector of goals to achieve|
| `start` | `geometry_msgs/PoseStamped` | Starting pose for path planning |
| `planner_id` | `string` | Name of the planner plugin to use for path planning |
| `use_start` | `bool` | If false, use current robot pose as path start, if true, use start above instead |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `planning_time` | `builtin_interfaces/Duration` | Time spent in path planning phase |
| `error_code` | `uint16` | Error code indicating the result status. Possible values: NONE, UNKNOWN, INVALID_PLANNER, TF_ERROR, START_OUTSIDE_MAP, GOAL_OUTSIDE_MAP, START_OCCUPIED, GOAL_OCCUPIED, TIMEOUT, NO_VALID_PATH, NO_VIAPOINTS_GIVEN|
| `error_msg` | `string` | Human readable error message that corresponds to the error code, when set|


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
        goal_msg.planner_id = 'GridBased'
        goal_msg.use_start = False
        
        # Create goal poses
        from geometry_msgs.msg import PoseStamped
        
        goal1 = PoseStamped()
        goal1.header.frame_id = 'map'
        goal1.header.stamp = self.get_clock().now().to_msg()
        goal1.pose.position.x = 2.0
        goal1.pose.position.y = 1.0
        goal1.pose.orientation.w = 1.0
        
        goal2 = PoseStamped()
        goal2.header.frame_id = 'map'
        goal2.header.stamp = self.get_clock().now().to_msg()
        goal2.pose.position.x = 4.0
        goal2.pose.position.y = 2.0
        goal2.pose.orientation.w = 1.0
        
        goal_msg.goals = [goal1, goal2]
        
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
        goal_msg.planner_id = "GridBased";
        goal_msg.use_start = false;
        
        // Create goal poses
        geometry_msgs::msg::PoseStamped goal1, goal2;
        
        goal1.header.frame_id = "map";
        goal1.header.stamp = this->now();
        goal1.pose.position.x = 2.0;
        goal1.pose.position.y = 1.0;
        goal1.pose.orientation.w = 1.0;
        
        goal2.header.frame_id = "map";
        goal2.header.stamp = this->now();
        goal2.pose.position.x = 4.0;
        goal2.pose.position.y = 2.0;
        goal2.pose.orientation.w = 1.0;
        
        goal_msg.goals = {goal1, goal2};
        
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
