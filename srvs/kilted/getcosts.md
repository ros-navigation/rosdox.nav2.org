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

class GetCostsClient(Node):
    def __init__(self):
        super().__init__('getcosts_client')
        self.client = self.create_client(GetCosts, 'getcosts')
        
    def send_request(self):
        request = GetCosts.Request()
        # Set request parameters here based on service definition
        
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

class GetCostsClient : public rclcpp::Node
{
public:
    GetCostsClient() : Node("getcosts_client")
    {
        client_ = create_client<nav2_msgs::srv::GetCosts>("getcosts");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::GetCosts::Request>();
        // Set request parameters here based on service definition

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

- [All Costmap Services](/kilted/srvs/index.html#costmap-services)
- [Service API Overview](/kilted/srvs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)