---
layout: default
title: "GetCostmap Service"
permalink: /srvs/rolling/getcostmap.html
---

# GetCostmap Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Retrieve the entire costmap as an occupancy grid

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `specs` | `nav2_msgs/CostmapMetaData` | Get the costmap Specifications for the requested costmap |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `map` | `nav2_msgs/Costmap` | Map data as occupancy grid |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCostmap

class GetCostmapClient(Node):
    def __init__(self):
        super().__init__('get_costmap_client')
        self.client = self.create_client(GetCostmap, 'get_costmap')
        
    def send_request(self):
        request = GetCostmap.Request()
        # Set request.specs as needed
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = GetCostmapClient()
    
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
#include "nav2_msgs/srv/get_costmap.hpp"

class GetCostmapClient : public rclcpp::Node
{
public:
    GetCostmapClient() : Node("get_costmap_client")
    {
        client_ = create_client<nav2_msgs::srv::GetCostmap>("get_costmap");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
        // Set request->specs as needed

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
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr client_;
};
```

## Related Services

- [All Costmap Services](/rolling/srvs/index.html#costmap-services)
- [Service API Overview](/rolling/srvs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
