---
layout: default
title: "SetInitialPose Service"
permalink: /srvs/humble/setinitialpose.html
---

# SetInitialPose Service

**Package:** `nav2_msgs`  
**Category:** Localization Services

Set the initial pose estimate for robot localization

## Message Definitions

### Request Message

| Field | Type | Description |
|-------|------|-------------|
| `pose` | `geometry_msgs/PoseWithCovarianceStamped` | Target pose for the service operation |


### Response Message

No fields defined.


## Usage Examples

### Python

```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import SetInitialPose

class SetInitialPoseClient(Node):
    def __init__(self):
        super().__init__('set_initial_pose_client')
        self.client = self.create_client(SetInitialPose, 'set_initial_pose')
        
    def send_request(self):
        request = SetInitialPose.Request()
        # Set request.pose as needed
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = SetInitialPoseClient()
    
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
#include "nav2_msgs/srv/set_initial_pose.hpp"

class SetInitialPoseClient : public rclcpp::Node
{
public:
    SetInitialPoseClient() : Node("set_initial_pose_client")
    {
        client_ = create_client<nav2_msgs::srv::SetInitialPose>("set_initial_pose");
    }

    void send_request()
    {
        auto request = std::make_shared<nav2_msgs::srv::SetInitialPose::Request>();
        // Set request->pose as needed

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
    rclcpp::Client<nav2_msgs::srv::SetInitialPose>::SharedPtr client_;
};
```

## Related Services

- [All Localization Services](/humble/srvs/index.html#localization-services)
- [Service API Overview](/humble/srvs/index.html)
- [Nav2 C++ API Documentation](/humble/html/index.html)
