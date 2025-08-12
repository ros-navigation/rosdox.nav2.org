---
layout: default
title: "ClearCostmapAroundPose Service"
permalink: /srvs/rolling/clearcostmaparoundpose.html
---

# ClearCostmapAroundPose Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Clear costmap within a specified distance around a given pose

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `pose` | `geometry_msgs/PoseStamped` | Clears the costmap within a distance around a given pose |
| `reset_distance` | `float64` | Distance parameter for clearing operations |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `response` | `std_msgs/Empty` | Response message or data |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearCostmapAroundPose

class ClearCostmapAroundPoseClient(Node):
    def __init__(self):
        super().__init__('clear_costmap_around_pose_client')
        self.client = self.create_client(ClearCostmapAroundPose, 'clear_costmap_around_pose')
        
    def send_request(self):
        request = ClearCostmapAroundPose.Request()
        request.pose.header.frame_id = 'map'
        request.pose.header.stamp = self.get_clock().now().to_msg()
        request.pose.pose.position.x = 0.0
        request.pose.pose.orientation.w = 1.0
        request.reset_distance = 0.0
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ClearCostmapAroundPoseClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Service call completed')
    else:
        client.get_logger().error('Service call failed')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clear_costmap_around_pose.hpp"

class ClearCostmapAroundPoseClient : public rclcpp::Node
{
public:
    ClearCostmapAroundPoseClient() : Node("clear_costmap_around_pose_client")
    {
        client_ = create_client<nav2_msgs::srv::ClearCostmapAroundPose>("clear_costmap_around_pose");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ClearCostmapAroundPose::Request>();
        request->pose.header.frame_id = "map";
        request->pose.header.stamp = this->now();
        request->pose.pose.position.x = 0.0;
        request->pose.pose.orientation.w = 1.0;
        request->reset_distance = 0.0;

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Service call completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Service call failed");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundPose>::SharedPtr client_;
};
```

## Related Services

- [All Costmap Services](/srvs/rolling/index.html#costmap-services)
- [Service API Overview](/srvs/rolling/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
