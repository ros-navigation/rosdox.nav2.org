---
layout: default
title: "DynamicEdges Service"
permalink: /srvs/jazzy/dynamicedges.html
---

# DynamicEdges Service

**Package:** `nav2_msgs`  
**Category:** Nav2 Route Services

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
| `success` | `bool` | Whether edge modifications were successful |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import DynamicEdges

class DynamicEdgesClient(Node):
    def __init__(self):
        super().__init__('dynamicedges_client')
        self.client = self.create_client(DynamicEdges, 'dynamicedges')
        
    def send_request(self):
        request = DynamicEdges.Request()
        # Set request parameters here based on service definition
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = DynamicEdgesClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Modify route graph edges dynamically completed')
    else:
        client.get_logger().error('Failed to modify route graph edges dynamically')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/dynamicedges.hpp"

class DynamicEdgesClient : public rclcpp::Node
{
public:
    DynamicEdgesClient() : Node("dynamicedges_client")
    {
        client_ = create_client<nav2_msgs::srv::DynamicEdges>("dynamicedges");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::DynamicEdges::Request>();
        // Set request parameters here based on service definition

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Modify route graph edges dynamically completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to modify route graph edges dynamically");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::DynamicEdges>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<DynamicEdgesClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Nav2 Route Services](/jazzy/srvs/index.html#nav2-route-services)
- [Service API Overview](/jazzy/srvs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)