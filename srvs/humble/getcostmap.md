---
layout: default
title: "GetCostmap Service"
permalink: /srvs/humble/getcostmap.html
---

# GetCostmap Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Retrieve the entire costmap as an occupancy grid

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `specs` | `nav2_msgs/CostmapMetaData` | Specifications for the requested costmap |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `map` | `nav2_msgs/Costmap` | The requested costmap data |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCostmap

class GetCostmapClient(Node):
    def __init__(self):
        super().__init__('getcostmap_client')
        self.client = self.create_client(GetCostmap, 'getcostmap')
        
    def send_request(self):
        request = GetCostmap.Request()
        # Set request parameters here based on service definition
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = GetCostmapClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Retrieve the complete costmap completed')
    else:
        client.get_logger().error('Failed to retrieve the complete costmap')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/getcostmap.hpp"

class GetCostmapClient : public rclcpp::Node
{
public:
    GetCostmapClient() : Node("getcostmap_client")
    {
        client_ = create_client<nav2_msgs::srv::GetCostmap>("getcostmap");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::GetCostmap::Request>();
        // Set request parameters here based on service definition

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Retrieve the complete costmap completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to retrieve the complete costmap");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::GetCostmap>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<GetCostmapClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Costmap Services](/humble/srvs/index.html#costmap-services)
- [Service API Overview](/humble/srvs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)