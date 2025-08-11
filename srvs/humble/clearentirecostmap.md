---
layout: default
title: "ClearEntireCostmap Service"
permalink: /srvs/humble/clearentirecostmap.html
---

# ClearEntireCostmap Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Clear the entire costmap, resetting all cells to free space

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `request` | `std_msgs/Empty` | Empty request |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `response` | `std_msgs/Empty` | Empty response indicating completion |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearEntireCostmap

class ClearEntireCostmapClient(Node):
    def __init__(self):
        super().__init__('clearentirecostmap_client')
        self.client = self.create_client(ClearEntireCostmap, 'clearentirecostmap')
        
    def send_request(self):
        request = ClearEntireCostmap.Request()
        # Set request parameters here based on service definition
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ClearEntireCostmapClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Clear the entire costmap completed')
    else:
        client.get_logger().error('Failed to clear the entire costmap')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clearentirecostmap.hpp"

class ClearEntireCostmapClient : public rclcpp::Node
{
public:
    ClearEntireCostmapClient() : Node("clearentirecostmap_client")
    {
        client_ = create_client<nav2_msgs::srv::ClearEntireCostmap>("clearentirecostmap");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ClearEntireCostmap::Request>();
        // Set request parameters here based on service definition

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Clear the entire costmap completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to clear the entire costmap");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::ClearEntireCostmap>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ClearEntireCostmapClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Costmap Services](/humble/srvs/index.html#costmap-services)
- [Service API Overview](/humble/srvs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)