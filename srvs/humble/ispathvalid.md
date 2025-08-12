---
layout: default
title: "IsPathValid Service"
permalink: /srvs/humble/ispathvalid.html
---

# IsPathValid Service

**Package:** `nav2_msgs`  
**Category:** Validation Services

Validate whether a given path is collision-free and traversable

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `path` | `nav_msgs/Path` | Determine if the current path is still valid |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `is_valid` | `bool` | Whether the validation passed |
| `invalid_pose_indices` | `int32[]` | Indices of poses that failed validation |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import IsPathValid

class IsPathValidClient(Node):
    def __init__(self):
        super().__init__('is_path_valid_client')
        self.client = self.create_client(IsPathValid, 'is_path_valid')
        
    def send_request(self):
        request = IsPathValid.Request()
        # Set request.path as needed
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = IsPathValidClient()
    
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
#include "nav2_msgs/srv/is_path_valid.hpp"

class IsPathValidClient : public rclcpp::Node
{
public:
    IsPathValidClient() : Node("is_path_valid_client")
    {
        client_ = create_client<nav2_msgs::srv::IsPathValid>("is_path_valid");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::IsPathValid::Request>();
        // Set request->path as needed

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
    rclcpp::Client<nav2_msgs::srv::IsPathValid>::SharedPtr client_;
};
```

## Related Services

- [All Validation Services](/srvs/humble/index.html#validation-services)
- [Service API Overview](/srvs/humble/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
