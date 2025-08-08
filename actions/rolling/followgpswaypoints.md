---
layout: default
title: "FollowGPSWaypoints Action"
permalink: /actions/rolling/followgpswaypoints.html
---

# FollowGPSWaypoints Action

**Package:** `nav2_msgs`  
**Category:** Navigation

Navigate robot through GPS-based waypoints for outdoor navigation

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `number_of_loops` | `uint32` | Integer numeric value |
| `goal_index` | `uint32` | Integer numeric value |
| `gps_poses` | `geographic_msgs/GeoPose[]` | Parameter for the action (see Nav2 documentation) |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `NONE` | `uint16` | result definition Error codes Note: The expected priority order of the errors should match the message order |
| `UNKNOWN` | `uint16` | Integer numeric value |
| `TASK_EXECUTOR_FAILED` | `uint16` | Integer numeric value |
| `NO_WAYPOINTS_GIVEN` | `uint16` | Integer numeric value |
| `STOP_ON_MISSED_WAYPOINT` | `uint16` | Integer numeric value |
| `missed_waypoints` | `WaypointStatus[]` | Parameter for the action (see Nav2 documentation) |
| `error_code` | `int16` | Error code indicating the result status |
| `error_msg` | `string` | Text string parameter |


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `current_waypoint` | `uint32` | feedback |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowGPSWaypoints

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, FollowGPSWaypoints, 'follow_gps_waypoints')
        
    def send_goal(self):
        goal_msg = FollowGPSWaypoints.Goal()
        # Set appropriate fields for FollowGPSWaypoints
        
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
#include "nav2_msgs/action/follow_gps_waypoints.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using FollowGPSWaypointsAction = nav2_msgs::action::FollowGPSWaypoints;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowGPSWaypointsAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<FollowGPSWaypointsAction>(
            this, "follow_gps_waypoints");
    }

    void send_goal()
    {
        auto goal_msg = FollowGPSWaypointsAction::Goal();
        // Set appropriate fields for FollowGPSWaypoints
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<FollowGPSWaypointsAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<FollowGPSWaypointsAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const FollowGPSWaypointsAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Navigation Actions](/rolling/actions/index.html#navigation)
- [Action API Overview](/rolling/actions/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
