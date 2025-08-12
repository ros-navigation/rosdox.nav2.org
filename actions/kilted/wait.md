---
layout: default
title: "Wait Action"
permalink: /actions/kilted/wait.html
---

# Wait Action

**Package:** `nav2_msgs`  
**Category:** Behaviors

Pause robot operation for a specified duration or until condition is met

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `time` | `builtin_interfaces/Duration` | Duration to wait/pause robot motion before continuing |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `total_elapsed_time` | `builtin_interfaces/Duration` | Total time taken to complete the action |
| `error_code` | `uint16` | Numeric error code indicating specific failure reason (0=success, various codes for different failure types) |
| `error_msg` | `string` | Human-readable error message describing what went wrong during action execution |
| `NONE` | `uint16` | Success status code indicating the action completed without errors |
| `UNKNOWN` | `uint16` | Generic error code for unexpected or unclassified failures |
| `TIMEOUT` | `uint16` | Error code indicating the action exceeded its maximum allowed time |


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `time_left` | `builtin_interfaces/Duration` | Remaining wait time before the wait action completes, in seconds (counts down to zero) |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import Wait

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, Wait, 'wait')
        
    def send_goal(self):
        goal_msg = Wait.Goal()
        goal_msg.time = Duration(seconds=5.0)
        
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
#include "nav2_msgs/action/wait.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using WaitAction = nav2_msgs::action::Wait;
    using GoalHandle = rclcpp_action::ClientGoalHandle<WaitAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<WaitAction>(
            this, "wait");
    }

    void send_goal()
    {
        auto goal_msg = WaitAction::Goal();
        goal_msg.time = rclcpp::Duration::from_seconds(5.0);
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<WaitAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<WaitAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const WaitAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Behaviors Actions](/actions/kilted/index.html#behaviors)
- [Action API Overview](/actions/kilted/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
