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
| `start_id` | `uint16` | Unique identifier for the starting waypoint in the route graph |
| `start` | `geometry_msgs/PoseStamped` | Starting pose for path planning |
| `goal_id` | `uint16` | Unique identifier for the target waypoint in the route graph |
| `goal` | `geometry_msgs/PoseStamped` | Target goal pose for path planning |
| `use_start` | `bool` | Whether to use the start field or find the start pose in TF |
| `use_poses` | `bool` | Whether to use the poses or the IDs fields for request |


### Result Message

| Field | Type | Description |
|-------|------|-------------|
| `NONE` | `uint16` | Success status code indicating the action completed without errors |
| `UNKNOWN` | `uint16` | Generic error code for unexpected or unclassified failures |
| `TF_ERROR` | `uint16` | Error code indicating a transform/localization failure |
| `NO_VALID_GRAPH` | `uint16` | Error code indicating the route graph is invalid or has no connectivity |
| `INDETERMINANT_NODES_ON_GRAPH` | `uint16` | Error code indicating graph nodes have ambiguous or undefined relationships |
| `TIMEOUT` | `uint16` | Error code indicating the action exceeded its maximum allowed time |
| `NO_VALID_ROUTE` | `uint16` | Error code indicating no feasible route exists between the specified waypoints |
| `OPERATION_FAILED` | `uint16` | Error code indicating a general operation failure occurred during execution |
| `INVALID_EDGE_SCORER_USE` | `uint16` | Error code indicating the edge scorer plugin was used incorrectly |
| `execution_duration` | `builtin_interfaces/Duration` | Total time taken for the route computation and tracking operation to complete |
| `error_code` | `uint16` | Numeric error code indicating specific failure reason (0=success, various codes for different failure types) |
| `error_msg` | `string` | Human-readable error message describing what went wrong during action execution |


### Feedback Message

| Field | Type | Description |
|-------|------|-------------|
| `last_node_id` | `uint16` | Identifier of the last successfully reached node in the route graph during navigation |
| `next_node_id` | `uint16` | Identifier of the next node to be visited in the route graph during navigation |
| `current_edge_id` | `uint16` | Identifier of the current edge being traversed in the route graph |
| `route` | `Route` | Computed route with waypoints and metadata |
| `path` | `nav_msgs/Path` | Computed navigation path with poses and metadata |
| `operations_triggered` | `string[]` | List of navigation operations or behaviors that have been triggered during route execution |
| `rerouted` | `bool` | Flag indicating whether the route has been dynamically recalculated due to obstacles or changes |



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
        from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        
        # Create route as path
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = 'map'
        waypoint1.pose.position.x = 1.0
        waypoint1.pose.position.y = 1.0
        waypoint1.pose.orientation.w = 1.0
        
        waypoint2 = PoseStamped()
        waypoint2.header.frame_id = 'map'
        waypoint2.pose.position.x = 5.0
        waypoint2.pose.position.y = 3.0
        waypoint2.pose.orientation.w = 1.0
        
        path.poses = [waypoint1, waypoint2]
        goal_msg.route = path
        
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
        // Create route as path
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        
        geometry_msgs::msg::PoseStamped waypoint1, waypoint2;
        waypoint1.header.frame_id = "map";
        waypoint1.pose.position.x = 1.0;
        waypoint1.pose.position.y = 1.0;
        waypoint1.pose.orientation.w = 1.0;
        
        waypoint2.header.frame_id = "map";
        waypoint2.pose.position.x = 5.0;
        waypoint2.pose.position.y = 3.0;
        waypoint2.pose.orientation.w = 1.0;
        
        path.poses = {waypoint1, waypoint2};
        goal_msg.route = path;
        
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

- [All Planning Actions](/actions/rolling/index.html#planning)
- [Action API Overview](/actions/rolling/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
