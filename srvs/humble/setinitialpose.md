---
layout: default
title: "SetInitialPose Service"
permalink: /srvs/humble/setinitialpose.html
---

# SetInitialPose Service

**Package:** `nav2_msgs`  
**Category:** Other Services

Set the initial pose estimate for robot localization

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `pose` | `geometry_msgs/PoseWithCovarianceStamped` | Initial pose estimate with covariance |


### Response Message

| Field | Type | Description |
|-------|------|-------------|
| (none) | - | This service has no response fields |


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SetInitialPose

class SetInitialPoseClient(Node):
    def __init__(self):
        super().__init__('setinitialpose_client')
        self.client = self.create_client(SetInitialPose, 'setinitialpose')
        
    def send_request(self):
        request = SetInitialPose.Request()
        # Set request parameters here based on service definition
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = SetInitialPoseClient()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Set robot's initial pose estimate completed')
    else:
        client.get_logger().error('Failed to set robot's initial pose estimate')
        
    client.destroy_node()
    rclpy.shutdown()
```

### C++

```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/setinitialpose.hpp"

class SetInitialPoseClient : public rclcpp::Node
{
public:
    SetInitialPoseClient() : Node("setinitialpose_client")
    {
        client_ = create_client<nav2_msgs::srv::SetInitialPose>("setinitialpose");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
        // Set request parameters here based on service definition

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {
            RCLCPP_INFO(get_logger(), "Set robot's initial pose estimate completed");
        }
        else
        {
            RCLCPP_ERROR(get_logger(), "Failed to set robot's initial pose estimate");
        }
    }

private:
    rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr client_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<SetInitialPoseClient>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}
```

## Related Services

- [All Other Services](/humble/srvs/index.html#other-services)
- [Service API Overview](/humble/srvs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)