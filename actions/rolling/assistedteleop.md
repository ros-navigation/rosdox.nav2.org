---
layout: default
title: "AssistedTeleop Action"
permalink: /actions/rolling/assistedteleop.html
---

# AssistedTeleop Action

**Package:** `nav2_msgs`  
**Category:** Behaviors

Provide assisted teleoperation with collision avoidance and safety checks

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `time_allowance` | `builtin_interfaces/Duration` | Maximum time allowed for this action to complete |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `total_elapsed_time` | `builtin_interfaces/Duration` | Total time taken to complete the action |
| `error_code` | `uint16` | Error code indicating the result status. Possible values: NONE, UNKNOWN, TIMEOUT, TF_ERROR|
| `error_msg` | `string` | Human readable error message that corresponds to the error code, when set|


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `current_teleop_duration` | `builtin_interfaces/Duration` | feedback |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import AssistedTeleop

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, AssistedTeleop, 'assisted_teleop')
        
    def send_goal(self):
        goal_msg = AssistedTeleop.Goal()
        # Set appropriate fields for AssistedTeleop
        
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
#include "nav2_msgs/action/assisted_teleop.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using AssistedTeleopAction = nav2_msgs::action::AssistedTeleop;
    using GoalHandle = rclcpp_action::ClientGoalHandle<AssistedTeleopAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<AssistedTeleopAction>(
            this, "assisted_teleop");
    }

    void send_goal()
    {
        auto goal_msg = AssistedTeleopAction::Goal();
        // Set appropriate fields for AssistedTeleop
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<AssistedTeleopAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<AssistedTeleopAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const AssistedTeleopAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Behaviors Actions](/rolling/actions/index.html#behaviors)
- [Action API Overview](/rolling/actions/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
