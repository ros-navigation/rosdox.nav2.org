---
layout: default
title: "ComputeAndTrackRoute Action"
permalink: /actions/rolling/computeandtrackroute.html
---

# ComputeAndTrackRoute Action

**Package:** `nav2_msgs`  
**Category:** Planning

Compute and actively track a route with dynamic replanning

## Message Definitions

### Goal Message

| Field | Type | Description |
|-------|------|-------------|
| `start_id` | `uint16` | ID of start node to use on the graph|
| `start` | `geometry_msgs/PoseStamped` | Starting pose for path planning |
| `goal_id` | `uint16` | ID of goal node to use on the graph|
| `goal` | `geometry_msgs/PoseStamped` | Target goal pose for path planning |
| `use_start` | `bool` | Whether to use the start field or find the start pose in TF |
| `use_poses` | `bool` | Whether to use the poses or the IDs fields for request |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `execution_duration` | `builtin_interfaces/Duration` | execution duration of the action|
| `error_code` | `uint16` | Error code indicating the result status. Possible values: NONE, UNKNOWN, TF_ERROR, NO_VALID_GRAPH, INDETERMINANT_NODES_ON_GRAPH, TIMEOUT, NO_VALID_ROUTE, OPERATION_FAILED, INVALID_EDGE_SCORER_USE|
| `error_msg` | `string` | Human readable error message that corresponds to the error code, when set|


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `last_node_id` | `uint16` | The last node achieved on the route|
| `next_node_id` | `uint16` | The next node to be achieved following the current edge|
| `current_edge_id` | `uint16` | The current edge being followed|
| `route` | `Route` | Computed route with waypoints and metadata |
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `operations_triggered` | `string[]` | Parameter for the action (see Nav2 documentation) |
| `rerouted` | `bool` | Whether a rerouting request was processed|



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import ComputeAndTrackRoute

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, ComputeAndTrackRoute, 'compute_and_track_route')
        
    def send_goal(self):
        goal_msg = ComputeAndTrackRoute.Goal()
        goal_msg.start_id = 1
        goal_msg.goal_id = 5
        
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
#include "nav2_msgs/action/compute_and_track_route.hpp"

class Nav2ActionClient : public rclcpp::Node
{
public:
    using ComputeAndTrackRouteAction = nav2_msgs::action::ComputeAndTrackRoute;
    using GoalHandle = rclcpp_action::ClientGoalHandle<ComputeAndTrackRouteAction>;

    Nav2ActionClient() : Node("nav2_action_client")
    {
        action_client_ = rclcpp_action::create_client<ComputeAndTrackRouteAction>(
            this, "compute_and_track_route");
    }

    void send_goal()
    {
        auto goal_msg = ComputeAndTrackRouteAction::Goal();
        goal_msg.start_id = 1;
        goal_msg.goal_id = 5;
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<ComputeAndTrackRouteAction>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

private:
    rclcpp_action::Client<ComputeAndTrackRouteAction>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const ComputeAndTrackRouteAction::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }
};
```

## Related Actions

- [All Planning Actions](/rolling/actions/index.html#planning)
- [Action API Overview](/rolling/actions/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
