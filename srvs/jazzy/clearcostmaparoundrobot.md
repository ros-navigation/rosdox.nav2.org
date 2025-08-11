---
layout: default
title: "ClearCostmapAroundRobot Service"
permalink: /srvs/jazzy/clearcostmaparoundrobot.html
---

# ClearCostmapAroundRobot Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Clear costmap within a specified distance around the robot's current position

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `reset_distance` | `float32` | Distance radius in meters within which to clear costmap cells around robot |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `response` | `std_msgs/Empty` | Empty response indicating completion |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearCostmapAroundRobot

class ClearCostmapAroundRobotClient(Node):
    def __init__(self):
        super().__init__('clearcostmaparoundrobot_client')
        self.client = self.create_client(ClearCostmapAroundRobot, 'clearcostmaparoundrobot')
        
    def send_request(self):
        request = ClearCostmapAroundRobot.Request()
        # Set request parameters here based on service definition
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ClearCostmapAroundRobotClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Clear costmap around robot's current position completed')
    else:
        client.get_logger().error('Failed to clear costmap around robot's current position')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clearcostmaparoundrobot.hpp"

class ClearCostmapAroundRobotClient : public rclcpp::Node
{
public:
    ClearCostmapAroundRobotClient() : Node("clearcostmaparoundrobot_client")
    {
        client_ = create_client<nav2_msgs::srv::ClearCostmapAroundRobot>("clearcostmaparoundrobot");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ClearCostmapAroundRobot::Request>();
        // Set request parameters here based on service definition

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Clear costmap around robot's current position completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to clear costmap around robot's current position");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ClearCostmapAroundRobotClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Costmap Services](/jazzy/srvs/index.html#costmap-services)
- [Service API Overview](/jazzy/srvs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)