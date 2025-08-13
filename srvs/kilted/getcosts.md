---
layout: default
title: "GetCosts Service"
permalink: /srvs/kilted/getcosts.html
---

# GetCosts Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Retrieve costmap cost values at specified poses

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `use_footprint` | `bool` | Get costmap costs at given poses |
| `poses` | `geometry_msgs/PoseStamped[]` | Array of poses to process |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `costs` | `float32[]` | Array of cost values |
| `success` | `bool` | Whether the operation completed successfully |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCosts

class GetCostsClient(Node):
    def __init__(self):
        super().__init__('get_costs_client')
        self.client = self.create_client(GetCosts, 'get_costs')
        
    def send_request(self):
        request = GetCosts.Request()
        request.use_footprint = True
        request.poses.header.frame_id = 'map'
        request.poses.header.stamp = self.get_clock().now().to_msg()
        request.poses.pose.position.x = 0.0
        request.poses.pose.orientation.w = 1.0
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = GetCostsClient()
    
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
#include "nav2_msgs/srv/get_costs.hpp"

class GetCostsClient : public rclcpp::Node
{
public:
    GetCostsClient() : Node("get_costs_client")
    {
        client_ = create_client<nav2_msgs::srv::GetCosts>("get_costs");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
        request->use_footprint = true;
        request->poses.header.frame_id = "map";
        request->poses.header.stamp = this->now();
        request->poses.pose.position.x = 0.0;
        request->poses.pose.orientation.w = 1.0;

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
    rclcpp::Client<nav2_msgs::srv::GetCosts>::SharedPtr client_;
};
```

## Related Services

- [All Costmap Services](/kilted/srvs/index.html#costmap-services)
- [Service API Overview](/kilted/srvs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
