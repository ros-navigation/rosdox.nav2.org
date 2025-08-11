---
layout: default
title: "IsPathValid Service"
permalink: /srvs/humble/ispathvalid.html
---

# IsPathValid Service

**Package:** `nav2_msgs`  
**Category:** Other Services

Validate whether a given path is collision-free and traversable

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `path` | `nav_msgs/Path` | Path to validate |
| `max_cost` | `uint8` | Maximum allowable cost (default: 254) |
| `consider_unknown_as_obstacle` | `bool` | Treat unknown cells as obstacles (default: false) |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `is_valid` | `bool` | Whether the path is valid |
| `invalid_pose_indices` | `int32[]` | Indices of invalid poses in the path |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import IsPathValid
from nav_msgs.msg import Path, OccupancyGrid

class IsPathValidClient(Node):
    def __init__(self):
        super().__init__('ispathvalid_client')
        self.client = self.create_client(IsPathValid, 'planner_server/is_path_valid')
        
    def send_request(self):
        request = IsPathValid.Request()
        # Create example path to validate
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        # Add path poses...
        request.path = path
        request.max_cost = 200
        request.consider_unknown_as_obstacle = False
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = IsPathValidClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Validate a navigation path completed')
    else:
        client.get_logger().error('Failed to validate a navigation path')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/ispathvalid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

class IsPathValidClient : public rclcpp::Node
{
public:
    IsPathValidClient() : Node("ispathvalid_client")
    {
        client_ = create_client<nav2_msgs::srv::IsPathValid>("planner_server/is_path_valid");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
        // Create example path to validate
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        // Add path poses...
        request->path = path;
        request->max_cost = 200;
        request->consider_unknown_as_obstacle = false;

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Validate a navigation path completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to validate a navigation path");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<IsPathValidClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Other Services](/humble/srvs/index.html#other-services)
- [Service API Overview](/humble/srvs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)