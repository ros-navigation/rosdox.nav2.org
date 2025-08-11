---
layout: default
title: "DockRobot Action"
permalink: /actions/rolling/dockrobot.html
---

# DockRobot Action

**Package:** `nav2_msgs`  
**Category:** Autodocking

Autonomously dock robot to a charging station or docking platform

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `use_dock_id` | `bool` | goal definition. Whether to use the dock_id or dock_pose fields |
| `dock_id` | `string` | Dock name or ID to dock at, from given dock database |
| `dock_pose` | `geometry_msgs/PoseStamped` | Dock pose |
| `dock_type` | `string` | If using dock_pose, what type of dock it is. Not necessary if only using one type of dock. |
| `max_staging_time` | `float32` | Maximum time for navigation to get to the dock's staging pose. |
| `navigate_to_staging_pose` | `bool` | Whether or not to navigate to staging pose or assume robot is already at staging pose within tolerance to execute behavior |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | docking success status |
| `error_code` | `uint16` | Error code indicating the result status. Possible values: NONE, DOCK_NOT_IN_DB, DOCK_NOT_VALID, FAILED_TO_STAGE, FAILED_TO_DETECT_DOCK, FAILED_TO_CONTROL, FAILED_TO_CHARGE, TIMEOUT, UNKNOWN|
| `num_retries` | `uint16` | Number of retries attempted |
| `error_msg` | `string` | Text string parameter |


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `state` | `uint16` | Current docking state. Possible values: NAV_TO_STAGING_POSE, INITIAL_PERCEPTION, CONTROLLING, WAIT_FOR_CHARGE, RETRY|
| `docking_time` | `builtin_interfaces/Duration` | Docking time elapsed |
| `num_retries` | `uint16` | Number of retries attempted |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import DockRobot

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, DockRobot, 'dock_robot')
        
    def send_goal(self):
        goal_msg = DockRobot.Goal()
        goal_msg.use_dock_id = True
        goal_msg.dock_id = 'dock_01' 
        goal_msg.dock_type = 'nova_carter_dock' 
        goal_msg.max_staging_time = Duration(seconds=30.0)
        
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
#include "nav2_msgs/action/dock_robot.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using DockRobotAction = nav2_msgs::action::DockRobot;
    using GoalHandle = rclcpp_action::ClientGoalHandle<DockRobotAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<DockRobotAction>(
            this, "dock_robot");
    }

    void send_goal()
    {
        auto goal_msg = DockRobotAction::Goal();
        goal_msg.use_dock_id = true;
        goal_msg.dock_id = "dock_01";
        goal_msg.dock_type = "nova_carter_dock";
        goal_msg.max_staging_time = rclcpp::Duration::from_seconds(30.0);
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<DockRobotAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<DockRobotAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const DockRobotAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Autodocking Actions](/rolling/actions/index.html#autodocking)
- [Action API Overview](/rolling/actions/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
