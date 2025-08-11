---
layout: default
title: "ClearCostmapAroundPose Service"
permalink: /srvs/humble/clearcostmaparoundpose.html
---

# ClearCostmapAroundPose Service

**Package:** `nav2_msgs`  
**Category:** Costmap Services

Clear costmap within a specified distance around a given pose

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `pose` | `geometry_msgs/PoseStamped` | Target pose around which to clear the costmap |
| `reset_distance` | `float64` | Distance radius in meters within which to clear costmap cells |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| `response` | `std_msgs/Empty` | Empty response indicating completion |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ClearCostmapAroundPose

class ClearCostmapAroundPoseClient(Node):
    def __init__(self):
        super().__init__('clearcostmaparoundpose_client')
        self.client = self.create_client(ClearCostmapAroundPose, 'clearcostmaparoundpose')
        
    def send_request(self):
        request = ClearCostmapAroundPose.Request()
        # Set request parameters here based on service definition
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = ClearCostmapAroundPoseClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Clear costmap around a specific pose completed')
    else:
        client.get_logger().error('Failed to clear costmap around a specific pose')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/clearcostmaparoundpose.hpp"

class ClearCostmapAroundPoseClient : public rclcpp::Node
{
public:
    ClearCostmapAroundPoseClient() : Node("clearcostmaparoundpose_client")
    {
        client_ = create_client<nav2_msgs::srv::ClearCostmapAroundPose>("clearcostmaparoundpose");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::ClearCostmapAroundPose::Request>();
        // Set request parameters here based on service definition

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Clear costmap around a specific pose completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to clear costmap around a specific pose");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::ClearCostmapAroundPose>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<ClearCostmapAroundPoseClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Costmap Services](/humble/srvs/index.html#costmap-services)
- [Service API Overview](/humble/srvs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)