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
| `reset_distance` | `float32` | Clears the costmap within a distance |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `response` | `std_msgs/Empty` | Response message or data |



## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearCostmapAroundRobot

class ClearCostmapAroundRobotClient(Node):
    def __init__(self):
        super().__init__('clear_costmap_around_robot_client')
        self.client = self.create_client(ClearCostmapAroundRobot, 'clear_costmap_around_robot')
        
    def send_request(self):
        request = ClearCostmapAroundRobot.Request()
        request.reset_distance = 0.0
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ClearCostmapAroundRobotClient()
    
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
#include "nav2_msgs/srv/clear_costmap_around_robot.hpp"

class ClearCostmapAroundRobotClient : public rclcpp::Node
{
public:
    ClearCostmapAroundRobotClient() : Node("clear_costmap_around_robot_client")
    {
        client_ = create_client<nav2_msgs::srv::ClearCostmapAroundRobot>("clear_costmap_around_robot");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ClearCostmapAroundRobot::Request>();
        request->reset_distance = 0.0;

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
    rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundRobot>::SharedPtr client_;
};
```

## Related Services

- [All Costmap Services](/jazzy/srvs/index.html#costmap-services)
- [Service API Overview](/jazzy/srvs/index.html)
- [Nav2 C++ API Documentation](/jazzy/html/index.html)
