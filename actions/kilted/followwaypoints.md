---
layout: default
title: "FollowWaypoints Action"
permalink: /actions/kilted/followwaypoints.html
---

# FollowWaypoints Action

**Package:** `nav2_msgs`  
**Category:** Navigation

Navigate robot through a series of waypoints with optional task execution at each point

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `number_of_loops` | `uint32` | Integer numeric value |
| `goal_index` | `uint32` | Integer numeric value |
| `poses` | `geometry_msgs/PoseStamped[]` | Array of poses defining waypoints or path |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `NONE` | `uint16` | result definition Error codes Note: The expected priority order of the errors should match the message order |
| `UNKNOWN` | `uint16` | Integer numeric value |
| `TASK_EXECUTOR_FAILED` | `uint16` | Integer numeric value |
| `NO_VALID_WAYPOINTS` | `uint16` | Integer numeric value |
| `STOP_ON_MISSED_WAYPOINT` | `uint16` | Integer numeric value |
| `missed_waypoints` | `WaypointStatus[]` | Parameter for the action (see Nav2 documentation) |
| `error_code` | `uint16` | Error code indicating the result status |
| `error_msg` | `string` | Text string parameter |


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `current_waypoint` | `uint32` | Integer numeric value |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import FollowWaypoints

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
        
    def send_goal(self):
        goal_msg = FollowWaypoints.Goal()
        waypoint = geometry_msgs.msg.PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.pose.position.x = 2.0
        waypoint.pose.position.y = 1.5
        waypoint.pose.orientation.w = 1.0
        goal_msg.poses = [waypoint]
        
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
#include "nav2_msgs/action/follow_waypoints.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using FollowWaypointsAction = nav2_msgs::action::FollowWaypoints;
    using GoalHandle = rclcpp_action::ClientGoalHandle<FollowWaypointsAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<FollowWaypointsAction>(
            this, "follow_waypoints");
    }

    void send_goal()
    {
        auto goal_msg = FollowWaypointsAction::Goal();
        geometry_msgs::msg::PoseStamped waypoint;
        waypoint.header.frame_id = "map";
        waypoint.pose.position.x = 2.0;
        waypoint.pose.position.y = 1.5;
        waypoint.pose.orientation.w = 1.0;
        goal_msg.poses.push_back(waypoint);
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<FollowWaypointsAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<FollowWaypointsAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const FollowWaypointsAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Navigation Actions](/kilted/actions/index.html#navigation)
- [Action API Overview](/kilted/actions/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
