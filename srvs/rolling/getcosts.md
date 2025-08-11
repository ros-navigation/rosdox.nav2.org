---
layout: default
title: "GetCosts Service"
permalink: /srvs/rolling/getcosts.html
---

# GetCosts Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Retrieve costmap cost values at specified poses

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `use_footprint` | `bool` | Whether to use robot footprint for cost calculation or single point |
| `poses` | `geometry_msgs/PoseStamped[]` | Array of poses to query for cost values |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `costs` | `float32[]` | Array of cost values corresponding to input poses |
| `success` | `bool` | Whether the cost query was successful |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCosts
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped

class GetCostsClient(Node):
    def __init__(self):
        super().__init__('getcosts_client')
        self.client = self.create_client(GetCosts, 'global_costmap/get_cost_global_costmap')
        
    def send_request(self):
        request = GetCosts.Request()
        request.use_footprint = True
        # Create example poses to query
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.header.stamp = self.get_clock().now().to_msg()
        pose1.pose.position.x = 1.0
        pose1.pose.position.y = 2.0
        pose1.pose.orientation.w = 1.0
        
        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.header.stamp = self.get_clock().now().to_msg()
        pose2.pose.position.x = 3.0
        pose2.pose.position.y = 4.0
        pose2.pose.orientation.w = 1.0
        
        request.poses = [pose1, pose2]
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = GetCostsClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Retrieve cost values at specific poses completed')
    else:
        client.get_logger().error('Failed to retrieve cost values at specific poses')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/getcosts.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"

class GetCostsClient : public rclcpp::Node
{
public:
    GetCostsClient() : Node("getcosts_client")
    {
        client_ = create_client<nav2_msgs::srv::GetCosts>("global_costmap/get_cost_global_costmap");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
        request->use_footprint = true;
        // Create example poses to query
        geometry_msgs::msg::PoseStamped pose1, pose2;
        pose1.header.frame_id = "map";
        pose1.header.stamp = this->now();
        pose1.pose.position.x = 1.0;
        pose1.pose.position.y = 2.0;
        pose1.pose.orientation.w = 1.0;
        
        pose2.header.frame_id = "map";
        pose2.header.stamp = this->now();
        pose2.pose.position.x = 3.0;
        pose2.pose.position.y = 4.0;
        pose2.pose.orientation.w = 1.0;
        
        request->poses = {pose1, pose2};

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Retrieve cost values at specific poses completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to retrieve cost values at specific poses");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::GetCosts>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<GetCostsClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Costmap Services](/rolling/srvs/index.html#costmap-services)
- [Service API Overview](/rolling/srvs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)