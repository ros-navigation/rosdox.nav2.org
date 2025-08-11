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

def get_service_topic(service_name):
    """Map service names to their typical ROS topic names"""
    topic_map = {
        "clearcostmaparoundpose": "global_costmap/clear_around_global_costmap",
        "clearcostmaparoundrobot": "global_costmap/clear_around_global_costmap", 
        "clearcostmapexceptregion": "global_costmap/clear_except_global_costmap",
        "clearentirecostmap": "global_costmap/clear_entirely_global_costmap",
        "getcosts": "global_costmap/get_cost_global_costmap",
        "getcostmap": "global_costmap/get_costmap",
        "setroutegraph": "route_graph/set_route_graph",
        "dynamicedges": "route_graph/dynamic_edges",
        "loadmap": "map_server/load_map",
        "savemap": "map_server/save_map",
        "ispathvalid": "planner_server/is_path_valid",
        "managelifecyclenodes": "lifecycle_manager/manage_nodes",
        "reloaddockdatabase": "docking_server/reload_dock_database",
        "setinitialpose": "amcl/set_initial_pose"
    }
    return topic_map.get(service_name, service_name)

def get_python_imports(service_data):
    """Get additional Python imports needed for the service"""
    imports = ""
    for field, field_type, _ in service_data.get("request", []):
        if "geometry_msgs" in field_type:
            imports += "\nfrom geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped"
            break
    for field, field_type, _ in service_data.get("request", []):
        if "nav_msgs" in field_type:
            imports += "\nfrom nav_msgs.msg import Path, OccupancyGrid"
            break
    return imports

def get_cpp_includes(service_data):
    """Get additional C++ includes needed for the service"""
    includes = ""
    for field, field_type, _ in service_data.get("request", []):
        if "geometry_msgs" in field_type:
            includes += '\n#include "geometry_msgs/msg/pose_stamped.hpp"'
            includes += '\n#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"'
            break
    for field, field_type, _ in service_data.get("request", []):
        if "nav_msgs" in field_type:
            includes += '\n#include "nav_msgs/msg/path.hpp"'
            includes += '\n#include "nav_msgs/msg/occupancy_grid.hpp"'
            break
    return includes

def generate_python_request_code(service_data, service_name):
    """Generate Python code to populate request fields"""
    code_lines = []
    
    for field, field_type, _ in service_data.get("request", []):
        if field == "pose" and "PoseStamped" in field_type:
            code_lines.extend([
                "request.pose.header.frame_id = 'map'",
                "request.pose.header.stamp = self.get_clock().now().to_msg()",
                "request.pose.pose.position.x = 2.0",
                "request.pose.pose.position.y = 1.0",
                "request.pose.pose.position.z = 0.0",
                "request.pose.pose.orientation.w = 1.0"
            ])
        elif field == "pose" and "PoseWithCovarianceStamped" in field_type:
            code_lines.extend([
                "request.pose.header.frame_id = 'map'",
                "request.pose.header.stamp = self.get_clock().now().to_msg()",
                "request.pose.pose.pose.position.x = 0.0",
                "request.pose.pose.pose.position.y = 0.0",
                "request.pose.pose.pose.orientation.w = 1.0",
                "# Set covariance matrix (6x6 = 36 elements)",
                "request.pose.pose.covariance = [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,",
                "                               0.0, 0.25, 0.0, 0.0, 0.0, 0.0,",
                "                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,",
                "                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,",
                "                               0.0, 0.0, 0.0, 0.0, 0.0, 0.0,",
                "                               0.0, 0.0, 0.0, 0.0, 0.0, 0.068]"
            ])
        elif field == "poses" and "PoseStamped[]" in field_type:
            code_lines.extend([
                "# Create example poses to query",
                "pose1 = PoseStamped()",
                "pose1.header.frame_id = 'map'",
                "pose1.header.stamp = self.get_clock().now().to_msg()",
                "pose1.pose.position.x = 1.0",
                "pose1.pose.position.y = 2.0",
                "pose1.pose.orientation.w = 1.0",
                "",
                "pose2 = PoseStamped()",
                "pose2.header.frame_id = 'map'",
                "pose2.header.stamp = self.get_clock().now().to_msg()",
                "pose2.pose.position.x = 3.0",
                "pose2.pose.position.y = 4.0",
                "pose2.pose.orientation.w = 1.0",
                "",
                "request.poses = [pose1, pose2]"
            ])
        elif field == "use_footprint":
            code_lines.append("request.use_footprint = True")
        elif field == "reset_distance":
            code_lines.append("request.reset_distance = 2.0")
        elif field == "graph_filepath":
            code_lines.append('request.graph_filepath = "/path/to/route_graph.yaml"')
        elif field == "filepath":
            code_lines.append('request.filepath = "/path/to/dock_database.yaml"')
        elif field == "map_url":
            code_lines.append('request.map_url = "file:///path/to/map.yaml"')
        elif field == "map_topic":
            code_lines.append('request.map_topic = "/map"')
        elif field == "image_format":
            code_lines.append('request.image_format = "pgm"')
        elif field == "map_mode":
            code_lines.append('request.map_mode = "trinary"')
        elif field == "free_thresh":
            code_lines.append("request.free_thresh = 0.25")
        elif field == "occupied_thresh":
            code_lines.append("request.occupied_thresh = 0.65")
        elif field == "path":
            code_lines.extend([
                "# Create example path to validate",
                "path = Path()",
                "path.header.frame_id = 'map'",
                "path.header.stamp = self.get_clock().now().to_msg()",
                "# Add path poses...",
                "request.path = path"
            ])
        elif field == "max_cost":
            code_lines.append("request.max_cost = 200")
        elif field == "consider_unknown_as_obstacle":
            code_lines.append("request.consider_unknown_as_obstacle = False")
        elif field == "command":
            code_lines.extend([
                "# Lifecycle commands: STARTUP=0, PAUSE=1, RESUME=2, RESET=3, SHUTDOWN=4",
                "request.command = 0  # STARTUP"
            ])
        elif field == "closed_edges":
            code_lines.append("request.closed_edges = [1, 2, 5]  # Edge IDs to close")
        elif field == "opened_edges":
            code_lines.append("request.opened_edges = [3, 4]     # Edge IDs to open")
        elif field == "adjust_edges":
            code_lines.append("request.adjust_edges = []          # No cost adjustments")
        elif field == "specs":
            code_lines.extend([
                "# Specify costmap region to retrieve",
                "request.specs.size_x = 100",
                "request.specs.size_y = 100",
                "request.specs.resolution = 0.05"
            ])
    
    if not code_lines:
        code_lines.append("# No request parameters needed")
    
    return "\n        ".join(code_lines)

def generate_cpp_request_code(service_data, service_name):
    """Generate C++ code to populate request fields"""
    code_lines = []
    
    for field, field_type, _ in service_data.get("request", []):
        if field == "pose" and "PoseStamped" in field_type:
            code_lines.extend([
                "request->pose.header.frame_id = \"map\";",
                "request->pose.header.stamp = this->now();",
                "request->pose.pose.position.x = 2.0;",
                "request->pose.pose.position.y = 1.0;",
                "request->pose.pose.position.z = 0.0;",
                "request->pose.pose.orientation.w = 1.0;"
            ])
        elif field == "pose" and "PoseWithCovarianceStamped" in field_type:
            code_lines.extend([
                "request->pose.header.frame_id = \"map\";",
                "request->pose.header.stamp = this->now();",
                "request->pose.pose.pose.position.x = 0.0;",
                "request->pose.pose.pose.position.y = 0.0;",
                "request->pose.pose.pose.orientation.w = 1.0;",
                "// Set covariance matrix",
                "request->pose.pose.covariance[0] = 0.25;  // x variance",
                "request->pose.pose.covariance[7] = 0.25;  // y variance", 
                "request->pose.pose.covariance[35] = 0.068; // yaw variance"
            ])
        elif field == "poses" and "PoseStamped[]" in field_type:
            code_lines.extend([
                "// Create example poses to query",
                "geometry_msgs::msg::PoseStamped pose1, pose2;",
                "pose1.header.frame_id = \"map\";",
                "pose1.header.stamp = this->now();",
                "pose1.pose.position.x = 1.0;",
                "pose1.pose.position.y = 2.0;",
                "pose1.pose.orientation.w = 1.0;",
                "",
                "pose2.header.frame_id = \"map\";",
                "pose2.header.stamp = this->now();",
                "pose2.pose.position.x = 3.0;",
                "pose2.pose.position.y = 4.0;",
                "pose2.pose.orientation.w = 1.0;",
                "",
                "request->poses = {pose1, pose2};"
            ])
        elif field == "use_footprint":
            code_lines.append("request->use_footprint = true;")
        elif field == "reset_distance":
            code_lines.append("request->reset_distance = 2.0;")
        elif field == "graph_filepath":
            code_lines.append('request->graph_filepath = "/path/to/route_graph.yaml";')
        elif field == "filepath":
            code_lines.append('request->filepath = "/path/to/dock_database.yaml";')
        elif field == "map_url":
            code_lines.append('request->map_url = "file:///path/to/map.yaml";')
        elif field == "map_topic":
            code_lines.append('request->map_topic = "/map";')
        elif field == "image_format":
            code_lines.append('request->image_format = "pgm";')
        elif field == "map_mode":
            code_lines.append('request->map_mode = "trinary";')
        elif field == "free_thresh":
            code_lines.append("request->free_thresh = 0.25;")
        elif field == "occupied_thresh":
            code_lines.append("request->occupied_thresh = 0.65;")
        elif field == "path":
            code_lines.extend([
                "// Create example path to validate",
                "nav_msgs::msg::Path path;",
                "path.header.frame_id = \"map\";",
                "path.header.stamp = this->now();",
                "// Add path poses...",
                "request->path = path;"
            ])
        elif field == "max_cost":
            code_lines.append("request->max_cost = 200;")
        elif field == "consider_unknown_as_obstacle":
            code_lines.append("request->consider_unknown_as_obstacle = false;")
        elif field == "command":
            code_lines.extend([
                "// Lifecycle commands: STARTUP=0, PAUSE=1, RESUME=2, RESET=3, SHUTDOWN=4",
                "request->command = 0;  // STARTUP"
            ])
        elif field == "closed_edges":
            code_lines.append("request->closed_edges = {1, 2, 5};  // Edge IDs to close")
        elif field == "opened_edges":
            code_lines.append("request->opened_edges = {3, 4};     // Edge IDs to open")
        elif field == "adjust_edges":
            code_lines.append("request->adjust_edges = {};          // No cost adjustments")
        elif field == "specs":
            code_lines.extend([
                "// Specify costmap region to retrieve",
                "request->specs.size_x = 100;",
                "request->specs.size_y = 100;",
                "request->specs.resolution = 0.05;"
            ])
    
    if not code_lines:
        code_lines.append("// No request parameters needed")
    
    return "\n        ".join(code_lines)

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
    
    # Generate Python request population based on service
    python_request_code = generate_python_request_code(service_data, service_name)
    cpp_request_code = generate_cpp_request_code(service_data, service_name)
    
    # Generate Python example
    service_class = service_data["title"]
    service_variable = service_name.lower()
    
    python_example = f"""```python
import rclpy
from rclpy.node import Node
from nav2_msgs.srv import {service_class}{get_python_imports(service_data)}

class {service_class}Client(Node):
    def __init__(self):
        super().__init__('{service_variable}_client')
        self.client = self.create_client({service_class}, '{get_service_topic(service_name)}')
        
    def send_request(self):
        request = {service_class}.Request()
        {python_request_code}
        
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
#include "nav2_msgs/srv/{service_name}.hpp"{get_cpp_includes(service_data)}

class {service_class}Client : public rclcpp::Node
{{
public:
    {service_class}Client() : Node("{service_variable}_client")
    {{
        client_ = create_client<nav2_msgs::srv::{service_class}>("{get_service_topic(service_name)}");
    }}

    void send_request()
    {{
        auto request = std::make_shared<nav2_msgs::srv::{service_class}::Request>();
        {cpp_request_code}

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