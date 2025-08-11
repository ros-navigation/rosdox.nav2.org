---
layout: default
title: "ComputeRoute Action"
permalink: /actions/kilted/computeroute.html
---

# ComputeRoute Action

**Package:** `nav2_msgs`  
**Category:** Planning

Compute a high-level route between waypoints using graph search

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `start_id` | `uint16` | Integer numeric value |
| `start` | `geometry_msgs/PoseStamped` | Starting pose for path planning |
| `goal_id` | `uint16` | Integer numeric value |
| `goal` | `geometry_msgs/PoseStamped` | Target goal pose for path planning |
| `use_start` | `bool` | Whether to use the start field or find the start pose in TF |
| `use_poses` | `bool` | Whether to use the poses or the IDs fields for request |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `planning_time` | `builtin_interfaces/Duration` | Time spent in path planning phase |
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `route` | `Route` | Computed route with waypoints and metadata |
| `error_code` | `uint16` | Error code indicating the result status. Possible values: NONE, UNKNOWN, TF_ERROR, NO_VALID_GRAPH, INDETERMINANT_NODES_ON_GRAPH, TIMEOUT, NO_VALID_ROUTE, INVALID_EDGE_SCORER_USE|
| `error_msg` | `string` | Human readable error message that corresponds to the error code, when set|


### Feedback Message

No fields defined.


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeRoute

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, ComputeRoute, 'compute_route')
        
    def send_goal(self):
        goal_msg = ComputeRoute.Goal()
        # Set appropriate fields for ComputeRoute
        
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
#include "nav2_msgs/action/compute_route.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using ComputeRouteAction = nav2_msgs::action::ComputeRoute;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputeRouteAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<ComputeRouteAction>(
            this, "compute_route");
    }

    void send_goal()
    {
        auto goal_msg = ComputeRouteAction::Goal();
        // Set appropriate fields for ComputeRoute
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<ComputeRouteAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<ComputeRouteAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const ComputeRouteAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Planning Actions](/kilted/actions/index.html#planning)
- [Action API Overview](/kilted/actions/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
