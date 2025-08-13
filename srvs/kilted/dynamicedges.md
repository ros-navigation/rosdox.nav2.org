---
layout: default
title: "DynamicEdges Service"
permalink: /srvs/kilted/dynamicedges.html
---

# DynamicEdges Service

**Package:** `nav2_msgs`  
**Category:** Route Services

Dynamically modify route graph edges during navigation

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `closed_edges` | `uint16[]` | Array of edge IDs to close |
| `opened_edges` | `uint16[]` | Array of edge IDs to open |
| `adjust_edges` | `EdgeCost[]` | Array of edges with modified costs |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the operation completed successfully |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import DynamicEdges

class DynamicEdgesClient(Node):
    def __init__(self):
        super().__init__('dynamic_edges_client')
        self.client = self.create_client(DynamicEdges, 'dynamic_edges')
        
    def send_request(self):
        request = DynamicEdges.Request()
        request.closed_edges = []  # Fill array as needed
        request.opened_edges = []  # Fill array as needed
        request.adjust_edges = []  # Fill array as needed
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = DynamicEdgesClient()
    
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
#include "nav2_msgs/srv/dynamic_edges.hpp"

class DynamicEdgesClient : public rclcpp::Node
{
public:
    DynamicEdgesClient() : Node("dynamic_edges_client")
    {
        client_ = create_client<nav2_msgs::srv::DynamicEdges>("dynamic_edges");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::DynamicEdges::Request>();
        // Fill request->closed_edges array as needed
        // Fill request->opened_edges array as needed
        // Fill request->adjust_edges array as needed

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
    rclcpp::Client<nav2_msgs::srv::DynamicEdges>::SharedPtr client_;
};
```

## Related Services

- [All Route Services](/kilted/srvs/index.html#route-services)
- [Service API Overview](/kilted/srvs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
