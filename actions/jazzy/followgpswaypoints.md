---
layout: default
title: "FollowGPSWaypoints Action"
permalink: /actions/jazzy/followgpswaypoints.html
---

# FollowGPSWaypoints Action

**Package:** `nav2_msgs`  
**Category:** Navigation

Navigate robot through GPS-based waypoints for outdoor navigation

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `number_of_loops` | `uint32` | Number of loops if the waypoints should be repeated|
| `goal_index` | `uint32` | The goal index to start following waypoints from, if not the start|
| `gps_poses` | `geographic_msgs/GeoPose[]` | Poses in GPS coordinates to follow|


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `missed_waypoints` | `MissedWaypoint[]` | The statuses of waypoints if missed|
| `error_code` | `int16` | Error code indicating the result status. Possible values: NONE, UNKNOWN, TASK_EXECUTOR_FAILED|
| `error_msg` | `string` | Human readable error message that corresponds to the error code, when set|


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `current_waypoint` | `uint32` | Current waypoint being executed|



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

- [All Navigation Actions](/jazzy/actions/index.html#navigation)
- [Action API Overview](/jazzy/actions/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
