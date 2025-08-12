---
layout: default
title: "ManageLifecycleNodes Service"
permalink: /srvs/jazzy/managelifecyclenodes.html
---

# ManageLifecycleNodes Service

**Package:** `nav2_msgs`  
**Category:** Lifecycle Services

Control lifecycle states of Nav2 nodes (startup, shutdown, pause, etc.)

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `STARTUP` = `0` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `PAUSE` = `1` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `RESUME` = `2` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `RESET` = `3` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `SHUTDOWN` = `4` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `CONFIGURE` = `5` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `CLEANUP` = `6` | `uint8` | Service parameter - see Nav2 documentation for specific usage details |
| `command` | `uint8` | Lifecycle command to execute |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the operation completed successfully |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes

class ManageLifecycleNodesClient(Node):
    def __init__(self):
        super().__init__('manage_lifecycle_nodes_client')
        self.client = self.create_client(ManageLifecycleNodes, 'manage_lifecycle_nodes')
        
    def send_request(self):
        request = ManageLifecycleNodes.Request()
        # Set request.command as needed
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ManageLifecycleNodesClient()
    
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
#include "nav2_msgs/srv/manage_lifecycle_nodes.hpp"

class ManageLifecycleNodesClient : public rclcpp::Node
{
public:
    ManageLifecycleNodesClient() : Node("manage_lifecycle_nodes_client")
    {
        client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("manage_lifecycle_nodes");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        // Set request->command as needed

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
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_;
};
```

## Related Services

- [All Lifecycle Services](/srvs/jazzy/index.html#lifecycle-services)
- [Service API Overview](/srvs/jazzy/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
