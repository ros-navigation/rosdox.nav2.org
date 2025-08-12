---
layout: default
title: "SetRouteGraph Service"
permalink: /srvs/rolling/setroutegraph.html
---

# SetRouteGraph Service

**Package:** `nav2_msgs`  
**Category:** Route Services

Load a new route graph from a specified file path

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `graph_filepath` | `string` | Path to route graph file |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the operation completed successfully |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SetRouteGraph

class SetRouteGraphClient(Node):
    def __init__(self):
        super().__init__('set_route_graph_client')
        self.client = self.create_client(SetRouteGraph, 'set_route_graph')
        
    def send_request(self):
        request = SetRouteGraph.Request()
        request.graph_filepath = '/path/to/file'
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = SetRouteGraphClient()
    
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
#include "nav2_msgs/srv/set_route_graph.hpp"

class SetRouteGraphClient : public rclcpp::Node
{
public:
    SetRouteGraphClient() : Node("set_route_graph_client")
    {
        client_ = create_client<nav2_msgs::srv::SetRouteGraph>("set_route_graph");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::SetRouteGraph::Request>();
        request->graph_filepath = "/path/to/file";

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
    rclcpp::Client<nav2_msgs::srv::SetRouteGraph>::SharedPtr client_;
};
```

## Related Services

- [All Route Services](/srvs/rolling/index.html#route-services)
- [Service API Overview](/srvs/rolling/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)
