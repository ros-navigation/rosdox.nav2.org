---
layout: default
title: "ManageLifecycleNodes Service"
permalink: /srvs/kilted/managelifecyclenodes.html
---

# ManageLifecycleNodes Service

**Package:** `nav2_msgs`  
**Category:** Other Services

Control lifecycle states of Nav2 nodes (startup, shutdown, pause, etc.)

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `command` | `uint8` | Lifecycle command (0=STARTUP, 1=PAUSE, 2=RESUME, 3=RESET, 4=SHUTDOWN, 5=CONFIGURE, 6=CLEANUP) |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the lifecycle command was executed successfully |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes

class ManageLifecycleNodesClient(Node):
    def __init__(self):
        super().__init__('managelifecyclenodes_client')
        self.client = self.create_client(ManageLifecycleNodes, 'lifecycle_manager/manage_nodes')
        
    def send_request(self):
        request = ManageLifecycleNodes.Request()
        # Lifecycle commands: STARTUP=0, PAUSE=1, RESUME=2, RESET=3, SHUTDOWN=4
        request.command = 0  # STARTUP
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ManageLifecycleNodesClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Control Nav2 node lifecycle completed')
    else:
        client.get_logger().error('Failed to control nav2 node lifecycle')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/managelifecyclenodes.hpp"

class ManageLifecycleNodesClient : public rclcpp::Node
{
public:
    ManageLifecycleNodesClient() : Node("managelifecyclenodes_client")
    {
        client_ = create_client<nav2_msgs::srv::ManageLifecycleNodes>("lifecycle_manager/manage_nodes");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ManageLifecycleNodes::Request>();
        // Lifecycle commands: STARTUP=0, PAUSE=1, RESUME=2, RESET=3, SHUTDOWN=4
        request->command = 0;  // STARTUP

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Control Nav2 node lifecycle completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to control nav2 node lifecycle");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::ManageLifecycleNodes>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ManageLifecycleNodesClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Other Services](/kilted/srvs/index.html#other-services)
- [Service API Overview](/kilted/srvs/index.html)
- [Nav2 C++ API Documentation](/kilted/html/index.html)