---
layout: default
title: "ClearCostmapExceptRegion Service"
permalink: /srvs/rolling/clearcostmapexceptregion.html
---

# ClearCostmapExceptRegion Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Clear entire costmap except for a specified rectangular region

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `reset_distance` | `float32` | Clears the costmap except a rectangular region specified by reset_distance |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `response` | `std_msgs/Empty` | Response message or data |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearCostmapExceptRegion

class ClearCostmapExceptRegionClient(Node):
    def __init__(self):
        super().__init__('clear_costmap_except_region_client')
        self.client = self.create_client(ClearCostmapExceptRegion, 'clear_costmap_except_region')
        
    def send_request(self):
        request = ClearCostmapExceptRegion.Request()
        request.reset_distance = 0.0
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ClearCostmapExceptRegionClient()
    
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
#include "nav2_msgs/srv/clear_costmap_except_region.hpp"

class ClearCostmapExceptRegionClient : public rclcpp::Node
{
public:
    ClearCostmapExceptRegionClient() : Node("clear_costmap_except_region_client")
    {
        client_ = create_client<nav2_msgs::srv::ClearCostmapExceptRegion>("clear_costmap_except_region");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ClearCostmapExceptRegion::Request>();
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
    rclcpp::Client<nav2_msgs::srv::ClearCostmapExceptRegion>::SharedPtr client_;
};
```

## Related Services

- [All Costmap Services](/rolling/srvs/index.html#costmap-services)
- [Service API Overview](/rolling/srvs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
