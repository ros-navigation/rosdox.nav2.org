---
layout: default
title: "SaveMap Service"
permalink: /srvs/humble/savemap.html
---

# SaveMap Service

**Package:** `nav2_msgs`  
**Category:** Map Services

Save the current map to a specified file location

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `map_topic` | `string` | URL of map resource Can be an absolute path to a file: file:///path/to/maps/floor1.yaml Or, relative to a ROS package: package://my_ros_package/maps/floor2.yaml |
| `map_url` | `string` | URL or file path to map file |
| `image_format` | `string` | Constants for image_format. Supported formats: pgm, png, bmp |
| `map_mode` | `string` | Map modes: trinary, scale or raw |
| `free_thresh` | `float32` | Thresholds. Values in range of [0.0 .. 1.0] |
| `occupied_thresh` | `float32` | Occupied space threshold value |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `result` | `bool` | Result code or status of the operation |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap

class SaveMapClient(Node):
    def __init__(self):
        super().__init__('save_map_client')
        self.client = self.create_client(SaveMap, 'save_map')
        
    def send_request(self):
        request = SaveMap.Request()
        request.map_topic = 'example_value'
        request.map_url = 'example_value'
        request.image_format = 'example_value'
        request.map_mode = 'example_value'
        request.free_thresh = 0.0
        request.occupied_thresh = 0.0
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = SaveMapClient()
    
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
#include "nav2_msgs/srv/save_map.hpp"

class SaveMapClient : public rclcpp::Node
{
public:
    SaveMapClient() : Node("save_map_client")
    {
        client_ = create_client<nav2_msgs::srv::SaveMap>("save_map");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
        request->map_topic = "example_value";
        request->map_url = "example_value";
        request->image_format = "example_value";
        request->map_mode = "example_value";
        request->free_thresh = 0.0;
        request->occupied_thresh = 0.0;

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
    rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr client_;
};
```

## Related Services

- [All Map Services](/srvs/humble/index.html#map-services)
- [Service API Overview](/srvs/humble/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
