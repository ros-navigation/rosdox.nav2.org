---
layout: default
title: "SetRouteGraph Service"
permalink: /srvs/humble/setroutegraph.html
---

# SetRouteGraph Service

**Package:** `nav2_msgs`  
**Category:** Nav2 Route Services

Load a new route graph from a specified file path

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `graph_filepath` | `string` | Path to the graph file to load |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the graph was loaded successfully |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SetRouteGraph

class SetRouteGraphClient(Node):
    def __init__(self):
        super().__init__('setroutegraph_client')
        self.client = self.create_client(SetRouteGraph, 'route_graph/set_route_graph')
        
    def send_request(self):
        request = SetRouteGraph.Request()
        request.graph_filepath = "/path/to/route_graph.yaml"
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = SetRouteGraphClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Load a new route graph completed')
    else:
        client.get_logger().error('Failed to load a new route graph')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/setroutegraph.hpp"

class SetRouteGraphClient : public rclcpp::Node
{
public:
    SetRouteGraphClient() : Node("setroutegraph_client")
    {
        client_ = create_client<nav2_msgs::srv::SetRouteGraph>("route_graph/set_route_graph");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::SetRouteGraph::Request>();
        request->graph_filepath = "/path/to/route_graph.yaml";

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Load a new route graph completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to load a new route graph");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::SetRouteGraph>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<SetRouteGraphClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Nav2 Route Services](/humble/srvs/index.html#nav2-route-services)
- [Service API Overview](/humble/srvs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)