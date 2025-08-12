---
layout: default
title: "ClearEntireCostmap Service"
permalink: /srvs/kilted/clearentirecostmap.html
---

# ClearEntireCostmap Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Clear the entire costmap, resetting all cells to free space

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `request` | `std_msgs/Empty` | Clears all layers on the costmap |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `response` | `std_msgs/Empty` | Response message or data |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap

class ClearEntireCostmapClient(Node):
    def __init__(self):
        super().__init__('clear_entire_costmap_client')
        self.client = self.create_client(ClearEntireCostmap, 'clear_entire_costmap')
        
    def send_request(self):
        request = ClearEntireCostmap.Request()
        # Set request.request as needed
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ClearEntireCostmapClient()
    
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
#include "nav2_msgs/srv/clear_entire_costmap.hpp"

class ClearEntireCostmapClient : public rclcpp::Node
{
public:
    ClearEntireCostmapClient() : Node("clear_entire_costmap_client")
    {
        client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>("clear_entire_costmap");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        // Set request->request as needed

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
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_;
};
```

## Related Services

- [All Costmap Services](/srvs/kilted/index.html#costmap-services)
- [Service API Overview](/srvs/kilted/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)
