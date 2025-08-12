---
layout: default
title: "ReloadDockDatabase Service"
permalink: /srvs/jazzy/reloaddockdatabase.html
---

# ReloadDockDatabase Service

**Package:** `nav2_msgs`  
**Category:** Docking Services

Reload the docking database with updated dock configurations

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `filepath` | `string` | Reloads the dock's database with a new filepath |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the operation completed successfully |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ReloadDockDatabase

class ReloadDockDatabaseClient(Node):
    def __init__(self):
        super().__init__('reload_dock_database_client')
        self.client = self.create_client(ReloadDockDatabase, 'reload_dock_database')
        
    def send_request(self):
        request = ReloadDockDatabase.Request()
        request.filepath = '/path/to/file'
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ReloadDockDatabaseClient()
    
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
#include "nav2_msgs/srv/reload_dock_database.hpp"

class ReloadDockDatabaseClient : public rclcpp::Node
{
public:
    ReloadDockDatabaseClient() : Node("reload_dock_database_client")
    {
        client_ = create_client<nav2_msgs::srv::ReloadDockDatabase>("reload_dock_database");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ReloadDockDatabase::Request>();
        request->filepath = "/path/to/file";

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
    rclcpp::Client<nav2_msgs::srv::ReloadDockDatabase>::SharedPtr client_;
};
```

## Related Services

- [All Docking Services](/srvs/jazzy/index.html#docking-services)
- [Service API Overview](/srvs/jazzy/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
