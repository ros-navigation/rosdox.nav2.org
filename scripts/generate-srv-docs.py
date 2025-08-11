#!/usr/bin/env python3

import os
from pathlib import Path

# Service definitions organized by category
service_definitions = {
    "costmap": {
        "clearcostmapexceptregion": {
            "title": "ClearCostmapExceptRegion",
            "description": "Clear entire costmap except for a specified rectangular region",
            "request": [
                ("reset_distance", "float32", "Distance defining the rectangular region to preserve")
            ],
            "response": [
                ("response", "std_msgs/Empty", "Empty response indicating completion")
            ],
            "usage_desc": "Clear all costmap cells except within a rectangular region",
            "example_params": "2.0"
        },
        "clearentirecostmap": {
            "title": "ClearEntireCostmap",
            "description": "Clear the entire costmap, resetting all cells to free space",
            "request": [
                ("request", "std_msgs/Empty", "Empty request")
            ],
            "response": [
                ("response", "std_msgs/Empty", "Empty response indicating completion")
            ],
            "usage_desc": "Clear the entire costmap",
            "example_params": ""
        },
        "getcostmap": {
            "title": "GetCostmap",
            "description": "Retrieve the entire costmap as an occupancy grid",
            "request": [
                ("specs", "nav2_msgs/CostmapMetaData", "Specifications for the requested costmap")
            ],
            "response": [
                ("map", "nav2_msgs/Costmap", "The requested costmap data")
            ],
            "usage_desc": "Retrieve the complete costmap",
            "example_params": ""
        }
    },
    "route": {
        "setroutegraph": {
            "title": "SetRouteGraph",
            "description": "Load a new route graph from a specified file path",
            "request": [
                ("graph_filepath", "string", "Path to the graph file to load")
            ],
            "response": [
                ("success", "bool", "Whether the graph was loaded successfully")
            ],
            "usage_desc": "Load a new route graph",
            "example_params": '"/path/to/graph.yaml"'
        },
        "dynamicedges": {
            "title": "DynamicEdges",
            "description": "Dynamically modify route graph edges during navigation",
            "request": [
                ("closed_edges", "uint16[]", "Array of edge IDs to close"),
                ("opened_edges", "uint16[]", "Array of edge IDs to open"),
                ("adjust_edges", "EdgeCost[]", "Array of edges with modified costs")
            ],
            "response": [
                ("success", "bool", "Whether edge modifications were successful")
            ],
            "usage_desc": "Modify route graph edges dynamically",
            "example_params": "[1, 2], [3, 4], []"
        }
    },
    "map": {
        "loadmap": {
            "title": "LoadMap",
            "description": "Load a map from a specified URL or file path",
            "request": [
                ("map_url", "string", "URL or file path to the map (file:///path/to/map.yaml or package://pkg/map.yaml)")
            ],
            "response": [
                ("map", "nav_msgs/OccupancyGrid", "The loaded map data"),
                ("result", "uint8", "Result code (0=SUCCESS, 1=MAP_DOES_NOT_EXIST, etc.)")
            ],
            "usage_desc": "Load a map from file",
            "example_params": '"file:///home/user/maps/floor1.yaml"'
        },
        "savemap": {
            "title": "SaveMap",
            "description": "Save the current map to a specified file location",
            "request": [
                ("map_topic", "string", "Topic name of the map to save"),
                ("map_url", "string", "URL or file path where to save the map"),
                ("image_format", "string", "Image format (pgm, png, bmp)"),
                ("map_mode", "string", "Map mode (trinary, scale, raw)"),
                ("free_thresh", "float32", "Free space threshold [0.0-1.0]"),
                ("occupied_thresh", "float32", "Occupied space threshold [0.0-1.0]")
            ],
            "response": [
                ("result", "bool", "Whether the map was saved successfully")
            ],
            "usage_desc": "Save current map to file",
            "example_params": '"/map", "file:///tmp/saved_map.yaml", "pgm", "trinary", 0.25, 0.65'
        }
    },
    "other": {
        "ispathvalid": {
            "title": "IsPathValid",
            "description": "Validate whether a given path is collision-free and traversable",
            "request": [
                ("path", "nav_msgs/Path", "Path to validate"),
                ("max_cost", "uint8", "Maximum allowable cost (default: 254)"),
                ("consider_unknown_as_obstacle", "bool", "Treat unknown cells as obstacles (default: false)")
            ],
            "response": [
                ("is_valid", "bool", "Whether the path is valid"),
                ("invalid_pose_indices", "int32[]", "Indices of invalid poses in the path")
            ],
            "usage_desc": "Validate a navigation path",
            "example_params": "path, 200, false"
        },
        "managelifecyclenodes": {
            "title": "ManageLifecycleNodes",
            "description": "Control lifecycle states of Nav2 nodes (startup, shutdown, pause, etc.)",
            "request": [
                ("command", "uint8", "Lifecycle command (0=STARTUP, 1=PAUSE, 2=RESUME, 3=RESET, 4=SHUTDOWN, 5=CONFIGURE, 6=CLEANUP)")
            ],
            "response": [
                ("success", "bool", "Whether the lifecycle command was executed successfully")
            ],
            "usage_desc": "Control Nav2 node lifecycle",
            "example_params": "0  # STARTUP"
        },
        "reloaddockdatabase": {
            "title": "ReloadDockDatabase",
            "description": "Reload the docking database with updated dock configurations",
            "request": [
                ("filepath", "string", "Path to the new dock database file")
            ],
            "response": [
                ("success", "bool", "Whether the database was reloaded successfully")
            ],
            "usage_desc": "Reload dock database",
            "example_params": '"/path/to/dock_database.yaml"'
        },
        "setinitialpose": {
            "title": "SetInitialPose",
            "description": "Set the initial pose estimate for robot localization",
            "request": [
                ("pose", "geometry_msgs/PoseWithCovarianceStamped", "Initial pose estimate with covariance")
            ],
            "response": [],
            "usage_desc": "Set robot's initial pose estimate",
            "example_params": "pose_with_covariance"
        }
    }
}

distributions = ["humble", "jazzy", "kilted", "rolling"]

def generate_service_page(service_name, service_data, distribution, category):
    """Generate a service documentation page"""
    
    # Create request table
    request_table = ""
    if service_data["request"]:
        request_table = "| Field | Type | Description |\n|-------|------|-------------|\n"
        for field, field_type, desc in service_data["request"]:
            request_table += f"| `{field}` | `{field_type}` | {desc} |\n"
    else:
        request_table = "| Field | Type | Description |\n|-------|------|-------------|\n| (none) | - | This service has no request fields |\n"
    
    # Create response table
    response_table = ""
    if service_data["response"]:
        response_table = "| Field | Type | Description |\n|-------|------|-------------|\n"
        for field, field_type, desc in service_data["response"]:
            response_table += f"| `{field}` | `{field_type}` | {desc} |\n"
    else:
        response_table = "| Field | Type | Description |\n|-------|------|-------------|\n| (none) | - | This service has no response fields |\n"
    
    # Generate Python example
    service_class = service_data["title"]
    service_variable = service_name.lower()
    
    python_example = f"""```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import {service_class}

class {service_class}Client(Node):
    def __init__(self):
        super().__init__('{service_variable}_client')
        self.client = self.create_client({service_class}, '{service_variable}')
        
    def send_request(self):
        request = {service_class}.Request()
        # Set request parameters here based on service definition
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = {service_class}Client()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('{service_data["usage_desc"]} completed')
    else:
        client.get_logger().error('Failed to {service_data["usage_desc"].lower()}')
        
    client.destroy_node()
    rclpy.shutdown()
```"""

    # Generate C++ example
    cpp_example = f"""```cpp
#include "rclcpp/rclcpp.hpp"
#include "nav2_msgs/srv/{service_name}.hpp"

class {service_class}Client : public rclcpp::Node
{{
public:
    {service_class}Client() : Node("{service_variable}_client")
    {{
        client_ = create_client<nav2_msgs::srv::{service_class}>("{service_variable}");
    }}

    void send_request()
    {{
        auto request = std::make_shared<nav2_msgs::srv::{service_class}::Request>();
        // Set request parameters here based on service definition

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {{
            RCLCPP_INFO(get_logger(), "{service_data["usage_desc"]} completed");
        }}
        else
        {{
            RCLCPP_ERROR(get_logger(), "Failed to {service_data["usage_desc"].lower()}");
        }}
    }}

private:
    rclcpp::Client<nav2_msgs::srv::{service_class}>::SharedPtr client_;
}};

int main(int argc, char ** argv)
{{
    rclcpp::init(argc, argv);
    auto client = std::make_shared<{service_class}Client>();
    
    client->send_request();
    
    rclcpp::shutdown();
    return 0;
}}
```"""

    # Category mapping for related services section
    category_section_map = {
        "costmap": "costmap-services",
        "route": "nav2-route-services", 
        "map": "map-services",
        "other": "other-services"
    }
    
    category_name_map = {
        "costmap": "Costmap Services",
        "route": "Nav2 Route Services",
        "map": "Map Services", 
        "other": "Other Services"
    }

    content = f"""---
layout: default
title: "{service_data['title']} Service"
permalink: /srvs/{distribution}/{service_name}.html
---

# {service_data['title']} Service

**Package:** `nav2_msgs`  
**Category:** {category_name_map[category]}

{service_data['description']}

## Message Definitions

### Request Message

{request_table}

### Response Message

{response_table}

## Usage Examples

### Python

{python_example}

### C++

{cpp_example}

## Related Services

- [All {category_name_map[category]}](/{distribution}/srvs/index.html#{category_section_map[category]})
- [Service API Overview](/{distribution}/srvs/index.html)
- [Nav2 C++ API Documentation](/{distribution}/html/index.html)"""

    return content

def main():
    # Create service directories for all distributions
    for dist in distributions:
        os.makedirs(f"srvs/{dist}", exist_ok=True)
    
    # Generate service pages for each distribution
    for dist in distributions:
        for category, services in service_definitions.items():
            for service_name, service_data in services.items():
                content = generate_service_page(service_name, service_data, dist, category)
                
                # Write the file
                filepath = f"srvs/{dist}/{service_name}.md"
                with open(filepath, 'w') as f:
                    f.write(content)
                
                print(f"Generated: {filepath}")
    
    # Also generate the missing services from the manual ones I created
    missing_services = {
        "clearcostmaparoundpose": {
            "title": "ClearCostmapAroundPose",
            "description": "Clear costmap within a specified distance around a given pose",
            "request": [
                ("pose", "geometry_msgs/PoseStamped", "Target pose around which to clear the costmap"),
                ("reset_distance", "float64", "Distance radius in meters within which to clear costmap cells")
            ],
            "response": [
                ("response", "std_msgs/Empty", "Empty response indicating completion")
            ],
            "usage_desc": "Clear costmap around a specific pose",
            "example_params": "pose, 2.0"
        },
        "clearcostmaparoundrobot": {
            "title": "ClearCostmapAroundRobot", 
            "description": "Clear costmap within a specified distance around the robot's current position",
            "request": [
                ("reset_distance", "float32", "Distance radius in meters within which to clear costmap cells around robot")
            ],
            "response": [
                ("response", "std_msgs/Empty", "Empty response indicating completion")
            ],
            "usage_desc": "Clear costmap around robot's current position",
            "example_params": "1.5"
        },
        "getcosts": {
            "title": "GetCosts",
            "description": "Retrieve costmap cost values at specified poses", 
            "request": [
                ("use_footprint", "bool", "Whether to use robot footprint for cost calculation or single point"),
                ("poses", "geometry_msgs/PoseStamped[]", "Array of poses to query for cost values")
            ],
            "response": [
                ("costs", "float32[]", "Array of cost values corresponding to input poses"),
                ("success", "bool", "Whether the cost query was successful")
            ],
            "usage_desc": "Retrieve cost values at specific poses",
            "example_params": "poses, true"
        }
    }
    
    # Generate the missing costmap services
    for dist in distributions:
        for service_name, service_data in missing_services.items():
            content = generate_service_page(service_name, service_data, dist, "costmap")
            
            # Write the file
            filepath = f"srvs/{dist}/{service_name}.md"
            with open(filepath, 'w') as f:
                f.write(content)
            
            print(f"Generated: {filepath}")

if __name__ == "__main__":
    main()