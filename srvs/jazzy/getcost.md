---
layout: default
title: "GetCost Service"
permalink: /srvs/jazzy/getcost.html
---

# GetCost Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Get costmap cost at given point

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `use_footprint` | `bool` | Get costmap cost at given point |
| `x` | `float32` | Floating point numerical value |
| `y` | `float32` | Floating point numerical value |
| `theta` | `float32` | Floating point numerical value |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `cost` | `float32` | Floating point numerical value |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import GetCost

class GetCostClient(Node):
    def __init__(self):
        super().__init__('get_cost_client')
        self.client = self.create_client(GetCost, 'get_cost')
        
    def send_request(self):
        request = GetCost.Request()
        request.use_footprint = True
        request.x = 0.0
        request.y = 0.0
        request.theta = 0.0
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = GetCostClient()
    
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
#include "nav2_msgs/srv/get_cost.hpp"

class GetCostClient : public rclcpp::Node
{
public:
    GetCostClient() : Node("get_cost_client")
    {
        client_ = create_client<nav2_msgs::srv::GetCost>("get_cost");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::GetCost::Request>();
        request->use_footprint = true;
        request->x = 0.0;
        request->y = 0.0;
        request->theta = 0.0;

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
    rclcpp::Client<nav2_msgs::srv::GetCost>::SharedPtr client_;
};
```

## Related Services

- [All Costmap Services](/srvs/jazzy/index.html#costmap-services)
- [Service API Overview](/srvs/jazzy/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
