---
layout: default
title: "ReloadDockDatabase Service"
permalink: /srvs/humble/reloaddockdatabase.html
---

# ReloadDockDatabase Service

**Package:** `nav2_msgs`  
**Category:** Other Services

Reload the docking database with updated dock configurations

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `filepath` | `string` | Path to the new dock database file |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the database was reloaded successfully |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ReloadDockDatabase

class ReloadDockDatabaseClient(Node):
    def __init__(self):
        super().__init__('reloaddockdatabase_client')
        self.client = self.create_client(ReloadDockDatabase, 'docking_server/reload_dock_database')
        
    def send_request(self):
        request = ReloadDockDatabase.Request()
        request.filepath = "/path/to/dock_database.yaml"
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ReloadDockDatabaseClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Reload dock database completed')
    else:
        client.get_logger().error('Failed to reload dock database')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/reloaddockdatabase.hpp"

class ReloadDockDatabaseClient : public rclcpp::Node
{
public:
    ReloadDockDatabaseClient() : Node("reloaddockdatabase_client")
    {
        client_ = create_client<nav2_msgs::srv::ReloadDockDatabase>("docking_server/reload_dock_database");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ReloadDockDatabase::Request>();
        request->filepath = "/path/to/dock_database.yaml";

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Reload dock database completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to reload dock database");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::ReloadDockDatabase>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ReloadDockDatabaseClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Other Services](/srvs/humble/index.html#other-services)
- [Service API Overview](/srvs/humble/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)