---
layout: default
title: "LoadMap Service"
permalink: /srvs/rolling/loadmap.html
---

# LoadMap Service

**Package:** `nav2_msgs`  
**Category:** Map Services

Load a map from a specified URL or file path

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `map_url` | `string` | URL or file path to the map (file:///path/to/map.yaml or package://pkg/map.yaml) |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `map` | `nav_msgs/OccupancyGrid` | The loaded map data |
| `result` | `uint8` | Result code (0=SUCCESS, 1=MAP_DOES_NOT_EXIST, etc.) |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap

class LoadMapClient(Node):
    def __init__(self):
        super().__init__('loadmap_client')
        self.client = self.create_client(LoadMap, 'map_server/load_map')
        
    def send_request(self):
        request = LoadMap.Request()
        request.map_url = "file:///path/to/map.yaml"
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = LoadMapClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Load a map from file completed')
    else:
        client.get_logger().error('Failed to load a map from file')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/loadmap.hpp"

class LoadMapClient : public rclcpp::Node
{
public:
    LoadMapClient() : Node("loadmap_client")
    {
        client_ = create_client<nav2_msgs::srv::LoadMap>("map_server/load_map");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        request->map_url = "file:///path/to/map.yaml";

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Load a map from file completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to load a map from file");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<LoadMapClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Map Services](/rolling/srvs/index.html#map-services)
- [Service API Overview](/rolling/srvs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)