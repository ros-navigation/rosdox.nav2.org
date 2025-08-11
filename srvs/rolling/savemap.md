---
layout: default
title: "SaveMap Service"
permalink: /srvs/rolling/savemap.html
---

# SaveMap Service

**Package:** `nav2_msgs`  
**Category:** Map Services

Save the current map to a specified file location

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `map_topic` | `string` | Topic name of the map to save |
| `map_url` | `string` | URL or file path where to save the map |
| `image_format` | `string` | Image format (pgm, png, bmp) |
| `map_mode` | `string` | Map mode (trinary, scale, raw) |
| `free_thresh` | `float32` | Free space threshold [0.0-1.0] |
| `occupied_thresh` | `float32` | Occupied space threshold [0.0-1.0] |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `result` | `bool` | Whether the map was saved successfully |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SaveMap

class SaveMapClient(Node):
    def __init__(self):
        super().__init__('savemap_client')
        self.client = self.create_client(SaveMap, 'map_server/save_map')
        
    def send_request(self):
        request = SaveMap.Request()
        request.map_topic = "/map"
        request.map_url = "file:///path/to/map.yaml"
        request.image_format = "pgm"
        request.map_mode = "trinary"
        request.free_thresh = 0.25
        request.occupied_thresh = 0.65
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = SaveMapClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Save current map to file completed')
    else:
        client.get_logger().error('Failed to save current map to file')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/savemap.hpp"

class SaveMapClient : public rclcpp::Node
{
public:
    SaveMapClient() : Node("savemap_client")
    {
        client_ = create_client<nav2_msgs::srv::SaveMap>("map_server/save_map");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::SaveMap::Request>();
        request->map_topic = "/map";
        request->map_url = "file:///path/to/map.yaml";
        request->image_format = "pgm";
        request->map_mode = "trinary";
        request->free_thresh = 0.25;
        request->occupied_thresh = 0.65;

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Save current map to file completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to save current map to file");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::SaveMap>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<SaveMapClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Map Services](/rolling/srvs/index.html#map-services)
- [Service API Overview](/rolling/srvs/index.html)
- [Nav2 C++ API Documentation](/rolling/html/index.html)