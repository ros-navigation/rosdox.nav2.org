---
layout: default
title: "LoadMap Service"
permalink: /srvs/humble/loadmap.html
---

# LoadMap Service

**Package:** `nav2_msgs`  
**Category:** Map Services

Load a map from a specified URL or file path

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `map_url` | `string` | URL of map resource Can be an absolute path to a file: file:///path/to/maps/floor1.yaml Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `RESULT_SUCCESS` = `0` | `uint8` | Result code defintions |
| `RESULT_MAP_DOES_NOT_EXIST` = `1` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `RESULT_INVALID_MAP_DATA` = `2` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `RESULT_INVALID_MAP_METADATA` = `3` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `RESULT_UNDEFINED_FAILURE` = `255` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `map` | `nav_msgs/OccupancyGrid` | Returned map is only valid if result equals RESULT_SUCCESS |
| `result` | `uint8` | Result code or status of the operation |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import LoadMap

class LoadMapClient(Node):
    def __init__(self):
        super().__init__('load_map_client')
        self.client = self.create_client(LoadMap, 'load_map')
        
    def send_request(self):
        request = LoadMap.Request()
        request.map_url = 'example_value'
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = LoadMapClient()
    
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
#include "nav2_msgs/srv/load_map.hpp"

class LoadMapClient : public rclcpp::Node
{
public:
    LoadMapClient() : Node("load_map_client")
    {
        client_ = create_client<nav2_msgs::srv::LoadMap>("load_map");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::LoadMap::Request>();
        request->map_url = "example_value";

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
    rclcpp::Client<nav2_msgs::srv::LoadMap>::SharedPtr client_;
};
```

## Related Services

- [All Map Services](/humble/srvs/index.html#map-services)
- [Service API Overview](/humble/srvs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
