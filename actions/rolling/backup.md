---
layout: default
title: "BackUp Action"
permalink: /actions/rolling/backup.html
---

# BackUp Action

**Package:** `nav2_msgs`  
**Category:** Behaviors

Move robot backwards a specified distance with obstacle detection

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `target` | `geometry_msgs/Point` | Target position or velocity vector |
| `speed` | `float32` | Speed for movement (m/s) |
| `time_allowance` | `builtin_interfaces/Duration` | Maximum time allowed for this action to complete |
| `disable_collision_checks` | `bool` | Boolean true/false flag |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `total_elapsed_time` | `builtin_interfaces/Duration` | Total time taken to complete the action |
| `error_code` | `uint16` | Error code indicating the result status. Possible values: NONE, UNKNOWN, TIMEOUT, TF_ERROR, INVALID_INPUT, COLLISION_AHEAD|
| `error_msg` | `string` | Human readable error message that corresponds to the error code, when set|


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `distance_traveled` | `float32` | Distance traveled so far|



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import BackUp

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, BackUp, 'back_up')
        
    def send_goal(self):
        goal_msg = BackUp.Goal()
        goal_msg.target.x = -0.5
        goal_msg.target.y = 0.0
        goal_msg.target.z = 0.0
        goal_msg.speed = 0.25
        goal_msg.time_allowance = Duration(seconds=10.0)
        
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
#include "nav2_msgs/action/back_up.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using BackUpAction = nav2_msgs::action::BackUp;
    using GoalHandle = rclcpp_action::ClientGoalHandle<BackUpAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<BackUpAction>(
            this, "back_up");
    }

    void send_goal()
    {
        auto goal_msg = BackUpAction::Goal();
        goal_msg.target.x = -0.5;
        goal_msg.target.y = 0.0;
        goal_msg.target.z = 0.0;
        goal_msg.speed = 0.25;
        goal_msg.time_allowance = rclcpp::Duration::from_seconds(10.0);
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<BackUpAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<BackUpAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const BackUpAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Behaviors Actions](/rolling/actions/index.html#behaviors)
- [Action API Overview](/rolling/actions/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
