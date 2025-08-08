---
layout: default
title: "UndockRobot Action"
permalink: /actions/kilted/undockrobot.html
---

# UndockRobot Action

**Package:** `nav2_msgs`  
**Category:** Autodocking

Autonomously undock robot from a charging station or docking platform

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `dock_type` | `string` | goal definition If initialized on a dock so the server doesn't know what type of dock its on, you must specify what dock it is to know where to stage for undocking. If only one type of dock plugin is present, it is not necessary to set. If not set & server instance was used to dock, server will use current dock information from last docking request. |
| `max_undocking_time` | `float32` | Maximum time to undock |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `NONE` | `uint16` | result definition Error codes Note: The expected priority order of the errors should match the message order |
| `DOCK_NOT_VALID` | `uint16` | Integer numeric value |
| `FAILED_TO_CONTROL` | `uint16` | Integer numeric value |
| `TIMEOUT` | `uint16` | Integer numeric value |
| `UNKNOWN` | `uint16` | Integer numeric value |
| `success` | `bool` | docking success status |
| `error_code` | `uint16` | Contextual error code, if any |
| `error_msg` | `string` | Text string parameter |


### Feedback Message

No fields defined.


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import UndockRobot

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, UndockRobot, 'undock_robot')
        
    def send_goal(self):
        goal_msg = UndockRobot.Goal()
        goal_msg.dock_type = 'nova_carter_dock' 
        
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
#include "nav2_msgs/action/undock_robot.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using UndockRobotAction = nav2_msgs::action::UndockRobot;
    using GoalHandle = rclcpp_action::ClientGoalHandle<UndockRobotAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<UndockRobotAction>(
            this, "undock_robot");
    }

    void send_goal()
    {
        auto goal_msg = UndockRobotAction::Goal();
        goal_msg.dock_type = "nova_carter_dock";
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<UndockRobotAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<UndockRobotAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const UndockRobotAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Autodocking Actions](/kilted/actions/index.html#autodocking)
- [Action API Overview](/kilted/actions/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
