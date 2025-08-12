#!/usr/bin/env python3
"""
Generate ROS Service API documentation for Nav2.
Parses .srv files from each distribution and generates Jekyll-compatible documentation.
"""

import os
import sys
import json
import argparse
from pathlib import Path
import re
from datetime import datetime
from typing import Dict, List, Any, Optional

class ServiceField:
    """Represents a field in a service request or response."""
    def __init__(self, type_name: str, field_name: str, comment: str = "", default_value: str = ""):
        self.type_name = type_name
        self.field_name = field_name
        self.comment = comment.strip()
        self.default_value = default_value.strip()
        self.is_array = "[]" in type_name
        self.base_type = type_name.replace("[]", "")

class ServiceMessage:
    """Represents a Request or Response message."""
    def __init__(self, name: str):
        self.name = name
        self.fields: List[ServiceField] = []

class ServiceDefinition:
    """Represents a complete .srv file."""
    def __init__(self, name: str, package: str, file_path: str):
        self.name = name
        self.package = package
        self.file_path = file_path
        self.request = ServiceMessage("Request")
        self.response = ServiceMessage("Response")
        self.description = self._get_description()
        self.category = self._determine_category()

    def _get_description(self) -> str:
        """Get appropriate description for the service based on its name."""
        descriptions = {
            # Costmap Services
            'ClearCostmapAroundPose': 'Clear costmap within a specified distance around a given pose',
            'ClearCostmapAroundRobot': 'Clear costmap within a specified distance around the robot\'s current position',
            'ClearCostmapExceptRegion': 'Clear entire costmap except for a specified rectangular region',
            'ClearEntireCostmap': 'Clear the entire costmap, resetting all cells to free space',
            'GetCostmap': 'Retrieve the entire costmap as an occupancy grid',
            'GetCosts': 'Retrieve costmap cost values at specified poses',
            
            # Route Services
            'SetRouteGraph': 'Load a new route graph from a specified file path',
            'DynamicEdges': 'Dynamically modify route graph edges during navigation',
            
            # Map Services
            'LoadMap': 'Load a map from a specified URL or file path',
            'SaveMap': 'Save the current map to a specified file location',
            
            # Other Services
            'IsPathValid': 'Validate whether a given path is collision-free and traversable',
            'ManageLifecycleNodes': 'Control lifecycle states of Nav2 nodes (startup, shutdown, pause, etc.)',
            'ReloadDockDatabase': 'Reload the docking database with updated dock configurations',
            'SetInitialPose': 'Set the initial pose estimate for robot localization'
        }
        
        return descriptions.get(self.name, 'Nav2 service for robotic navigation and system control')

    def _determine_category(self) -> str:
        """Determine service category based on name."""
        name_lower = self.name.lower()
        
        # Costmap Services
        if 'costmap' in name_lower or 'cost' in name_lower:
            return 'costmap'
        # Route Services
        elif 'route' in name_lower or 'edge' in name_lower or 'graph' in name_lower:
            return 'route'
        # Map Services
        elif 'map' in name_lower:
            return 'map'
        # Docking Services
        elif 'dock' in name_lower:
            return 'docking'
        # Lifecycle Services
        elif 'lifecycle' in name_lower or 'manage' in name_lower:
            return 'lifecycle'
        # Validation Services
        elif 'valid' in name_lower or 'path' in name_lower:
            return 'validation'
        # Localization Services
        elif 'pose' in name_lower or 'initial' in name_lower:
            return 'localization'
        # Fallback for any other services
        else:
            return 'other'

class ServiceParser:
    """Parser for ROS .srv files."""
    
    def __init__(self, source_dir: Path):
        self.source_dir = source_dir
        
    def find_service_files(self) -> List[Path]:
        """Find all .srv files in the source directory."""
        service_files = []
        
        # Look in nav2_msgs/srv directory
        nav2_msgs_dir = self.source_dir / "nav2_msgs" / "srv"
        if nav2_msgs_dir.exists():
            service_files.extend(nav2_msgs_dir.glob("*.srv"))
            
        # Also look for service files in other nav2 packages
        for pkg_dir in self.source_dir.glob("nav2_*"):
            if pkg_dir.is_dir():
                srv_dir = pkg_dir / "srv"
                if srv_dir.exists():
                    service_files.extend(srv_dir.glob("*.srv"))
        
        return service_files
    
    def parse_service_file(self, service_file: Path) -> Optional[ServiceDefinition]:
        """Parse a single .srv file."""
        try:
            with open(service_file, 'r') as f:
                content = f.read()
            
            # Extract package name from path
            package = self._get_package_name(service_file)
            service_name = service_file.stem
            
            service_def = ServiceDefinition(service_name, package, str(service_file))
            
            # Split content into Request and Response sections
            if '---' in content:
                sections = content.split('---', 1)
                service_def.request = self._parse_message_section(sections[0], "Request")
                if len(sections) > 1:
                    service_def.response = self._parse_message_section(sections[1], "Response")
            else:
                # Service with only request section
                service_def.request = self._parse_message_section(content, "Request")
            
            # Extract description from comments
            extracted_desc = self._extract_description(content)
            if not service_def.description or service_def.description == 'Nav2 service for robotic navigation and system control':
                service_def.description = extracted_desc if extracted_desc else service_def.description
            
            return service_def
            
        except Exception as e:
            print(f"Error parsing {service_file}: {e}")
            return None
    
    def _get_package_name(self, service_file: Path) -> str:
        """Extract package name from file path."""
        # Navigate up to find the package directory
        current = service_file.parent
        while current != self.source_dir:
            if (current / "package.xml").exists():
                return current.name
            current = current.parent
        return "nav2_msgs"  # default
    
    def _parse_message_section(self, section: str, section_name: str) -> ServiceMessage:
        """Parse a message section (Request or Response)."""
        message = ServiceMessage(section_name)
        lines = section.strip().split('\n')
        
        current_comment = ""
        for line in lines:
            line = line.strip()
            if not line:
                continue
                
            if line.startswith('#'):
                # Comment line
                comment_text = line[1:].strip()
                if current_comment:
                    current_comment += " " + comment_text
                else:
                    current_comment = comment_text
            else:
                # Field definition
                field = self._parse_field_line(line, current_comment)
                if field:
                    message.fields.append(field)
                current_comment = ""
        
        return message
    
    def _parse_field_line(self, line: str, comment: str = "") -> Optional[ServiceField]:
        """Parse a single field definition line."""
        # Handle different field definition patterns
        # Pattern 1: "string target_frame"
        # Pattern 2: "float64[] path"
        # Pattern 3: "uint8 NONE=0"
        # Pattern 4: "geometry_msgs/PoseStamped pose"
        
        # Check for constants (contain =)
        if '=' in line:
            match = re.match(r'^(\S+(?:\[\])?)\s+(\w+)\s*=\s*(.+)', line)
            if match:
                type_name = match.group(1)
                field_name = match.group(2)
                default_value = match.group(3)
                
                # Look for inline comments
                if '#' in line:
                    inline_comment = line.split('#', 1)[1].strip()
                    if comment:
                        comment = comment + ". " + inline_comment
                    else:
                        comment = inline_comment
                        
                return ServiceField(type_name, field_name, comment, default_value)
        else:
            # Regular field without default value
            match = re.match(r'^(\S+(?:\[\])?)\s+(\w+)', line)
            if match:
                type_name = match.group(1)
                field_name = match.group(2)
                
                # Look for inline comments
                if '#' in line:
                    inline_comment = line.split('#', 1)[1].strip()
                    if comment:
                        comment = comment + ". " + inline_comment
                    else:
                        comment = inline_comment
                        
                return ServiceField(type_name, field_name, comment)
        
        return None
    
    def _extract_description(self, content: str) -> str:
        """Extract description from top-level comments."""
        lines = content.split('\n')
        description_lines = []
        
        for line in lines:
            line = line.strip()
            if line.startswith('#'):
                desc_line = line[1:].strip()
                if desc_line:
                    description_lines.append(desc_line)
            elif line and not line.startswith('#'):
                # Stop at first non-comment, non-empty line
                break
        
        return ' '.join(description_lines)

class ServiceDocGenerator:
    """Generates Jekyll documentation from parsed services."""
    
    def __init__(self, distribution: str, output_dir: Path):
        self.distribution = distribution
        self.output_dir = output_dir
        self.field_descriptions = self._load_field_descriptions()
        
    def generate_docs(self, services: List[ServiceDefinition]):
        """Generate all documentation files."""
        # Create output directories
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Group services by category
        categories = self._group_by_category(services)
        
        # Generate overview page
        self._generate_overview(services, categories)
        
        # Generate individual service pages
        for service in services:
            self._generate_service_page(service)
    
    def _group_by_category(self, services: List[ServiceDefinition]) -> Dict[str, List[ServiceDefinition]]:
        """Group services by category."""
        categories = {}
        for service in services:
            if service.category not in categories:
                categories[service.category] = []
            categories[service.category].append(service)
        return categories
    
    def _generate_overview(self, services: List[ServiceDefinition], categories: Dict[str, List[ServiceDefinition]]):
        """Generate the main overview page."""
        content = f"""---
layout: default
title: "Nav2 Service APIs - {self.distribution.title()}"
permalink: /{self.distribution}/srvs/
---

# Nav2 Service APIs - {self.distribution.title()}

This page documents the ROS Service APIs available in Nav2 for the {self.distribution} distribution. These services provide request-response interfaces for navigation configuration, system control, and data queries.

## Available Services ({len(services)} total)

"""
        
        # Add category sections in desired order
        category_names = {
            'costmap': 'Costmap Services',
            'route': 'Route Planning Services',
            'map': 'Map Services',
            'docking': 'Docking Services',
            'lifecycle': 'Lifecycle Services',
            'validation': 'Validation Services',
            'localization': 'Localization Services',
            'other': 'Other Services'
        }
        
        # Define the desired order
        category_order = ['costmap', 'route', 'map', 'docking', 'lifecycle', 'validation', 'localization', 'other']
        
        # Process categories in the specified order
        for category in category_order:
            if category in categories:
                category_services = sorted(categories[category], key=lambda x: x.name)
                display_name = category_names.get(category, category.title() + ' Services')
                content += f"""
### {display_name}

<div class="service-grid">
"""
                for service in category_services:
                    content += f"""  <div class="service-card">
    <h4><a href="/srvs/{self.distribution}/{service.name.lower()}.html">{service.name}</a></h4>
    <p class="service-package">Package: {service.package}</p>
    <p class="service-description">{service.description or 'No description available.'}</p>
  </div>
"""
                content += "</div>\n"
        
        content += f"""

## Related Documentation

- [Nav2 Action APIs](/{self.distribution}/actions/index.html)
- [Nav2 Message APIs](/{self.distribution}/msgs/index.html)
- [Nav2 C++ API Documentation](/{self.distribution}/html/index.html)
- [Nav2 Official Documentation](https://nav2.org/)

*Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S UTC')}*"""
        
        # Write overview file to the correct location
        overview_dir = self.output_dir / self.distribution / "srvs"
        overview_dir.mkdir(parents=True, exist_ok=True)
        
        with open(overview_dir / "index.md", 'w') as f:
            f.write(content)
    
    def _generate_service_page(self, service: ServiceDefinition):
        """Generate documentation page for a single service."""
        content = f"""---
layout: default
title: "{service.name} Service"
permalink: /srvs/{self.distribution}/{service.name.lower()}.html
---

# {service.name} Service

**Package:** `{service.package}`  
**Category:** {service.category.replace('_', ' ').title()} Services

{service.description or "No description available."}

## Message Definitions

### Request Message

"""
        
        # Request section
        content += self._generate_message_table(service.request)
        
        # Response section  
        content += "\n### Response Message\n\n"
        content += self._generate_message_table(service.response)
        
        # Usage examples
        python_example, cpp_example = self._generate_usage_examples(service)
        content += f"""

## Usage Examples

### Python

```python
{python_example}
```

### C++

```cpp
{cpp_example}
```

## Related Services

"""
        
        # Add links to related services in same category
        content += f"- [All {service.category.replace('_', ' ').title()} Services](/{self.distribution}/srvs/index.html#{service.category.replace('_', '-')}-services)\n"
        content += f"- [Service API Overview](/{self.distribution}/srvs/index.html)\n"
        content += f"- [Nav2 C++ API Documentation](/{self.distribution}/html/index.html)\n"
        
        # Write service file
        service_dir = self.output_dir / "srvs" / self.distribution
        service_dir.mkdir(parents=True, exist_ok=True)
        
        service_file = service_dir / f"{service.name.lower()}.md"
        with open(service_file, 'w') as f:
            f.write(content)
    
    def _generate_message_table(self, message: ServiceMessage) -> str:
        """Generate markdown table for message fields."""
        if not message.fields:
            return "No fields defined.\n"
        
        table = "| Field | Type | Description |\n"
        table += "|-------|------|-------------|\n"
        
        for field in message.fields:
            description = self._get_field_description(field)
            field_display = f"`{field.field_name}`"
            if field.default_value:
                field_display += f" = `{field.default_value}`"
            table += f"| {field_display} | `{field.type_name}` | {description} |\n"
        
        return table + "\n"
    
    def _generate_usage_examples(self, service: ServiceDefinition) -> tuple:
        """Generate Python and C++ usage examples."""
        service_name = service.name
        package = service.package
        snake_name = self._to_snake_case(service_name)
        
        # Generate Python example
        python_example = f"""import rclpy
from rclpy.node import Node
from {package}.srv import {service_name}

class {service_name}Client(Node):
    def __init__(self):
        super().__init__('{snake_name}_client')
        self.client = self.create_client({service_name}, '{snake_name}')
        
    def send_request(self):
        request = {service_name}.Request()
        {self._get_python_request_population(service)}
        
        self.client.wait_for_service()
        future = self.client.call_async(request)
        return future

def main():
    rclpy.init()
    client = {service_name}Client()
    
    future = client.send_request()
    rclpy.spin_until_future_complete(client, future)
    
    if future.result():
        client.get_logger().info('Service call completed')
    else:
        client.get_logger().error('Service call failed')
        
    client.destroy_node()
    rclpy.shutdown()"""
        
        # Generate C++ example
        cpp_example = f"""#include "rclcpp/rclcpp.hpp"
#include "{package}/srv/{snake_name}.hpp"

class {service_name}Client : public rclcpp::Node
{{
public:
    {service_name}Client() : Node("{snake_name}_client")
    {{
        client_ = create_client<{package}::srv::{service_name}>("{snake_name}");
    }}

    void send_request()
    {{
        auto request = std::make_shared<{package}::srv::{service_name}::Request>();
        {self._get_cpp_request_population(service)}

        client_->wait_for_service();
        
        auto result_future = client_->async_send_request(request);
        if (rclcpp::spin_until_future_complete(shared_from_this(), result_future) ==
            rclcpp::FutureReturnCode::SUCCESS)
        {{
            RCLCPP_INFO(get_logger(), "Service call completed");
        }}
        else
        {{
            RCLCPP_ERROR(get_logger(), "Service call failed");
        }}
    }}

private:
    rclcpp::Client<{package}::srv::{service_name}>::SharedPtr client_;
}};"""
        
        return python_example, cpp_example
    
    def _get_python_request_population(self, service: ServiceDefinition) -> str:
        """Get Python code to populate request fields."""
        lines = []
        for field in service.request.fields:
            if field.default_value:
                continue  # Skip constants
            
            if 'Header' in field.type_name:
                lines.extend([
                    f"request.{field.field_name}.frame_id = 'map'",
                    f"request.{field.field_name}.stamp = self.get_clock().now().to_msg()"
                ])
            elif 'PoseStamped' in field.type_name:
                lines.extend([
                    f"request.{field.field_name}.header.frame_id = 'map'",
                    f"request.{field.field_name}.header.stamp = self.get_clock().now().to_msg()",
                    f"request.{field.field_name}.pose.position.x = 0.0",
                    f"request.{field.field_name}.pose.orientation.w = 1.0"
                ])
            elif field.type_name in ['string', 'std_msgs/String']:
                if 'path' in field.field_name.lower() or 'file' in field.field_name.lower():
                    lines.append(f"request.{field.field_name} = '/path/to/file'")
                else:
                    lines.append(f"request.{field.field_name} = 'example_value'")
            elif field.type_name in ['float32', 'float64']:
                lines.append(f"request.{field.field_name} = 0.0")
            elif field.type_name in ['int32', 'uint32', 'int16', 'uint16', 'int64', 'uint64']:
                lines.append(f"request.{field.field_name} = 0")
            elif field.type_name == 'bool':
                lines.append(f"request.{field.field_name} = True")
            elif field.is_array:
                lines.append(f"request.{field.field_name} = []  # Fill array as needed")
            else:
                lines.append(f"# Set request.{field.field_name} as needed")
        
        if not lines:
            lines.append("# No request parameters needed")
        
        return "\n        ".join(lines)
    
    def _get_cpp_request_population(self, service: ServiceDefinition) -> str:
        """Get C++ code to populate request fields."""
        lines = []
        for field in service.request.fields:
            if field.default_value:
                continue  # Skip constants
            
            if 'Header' in field.type_name:
                lines.extend([
                    f"request->{field.field_name}.frame_id = \"map\";",
                    f"request->{field.field_name}.stamp = this->now();"
                ])
            elif 'PoseStamped' in field.type_name:
                lines.extend([
                    f"request->{field.field_name}.header.frame_id = \"map\";",
                    f"request->{field.field_name}.header.stamp = this->now();",
                    f"request->{field.field_name}.pose.position.x = 0.0;",
                    f"request->{field.field_name}.pose.orientation.w = 1.0;"
                ])
            elif field.type_name in ['string', 'std_msgs/String']:
                if 'path' in field.field_name.lower() or 'file' in field.field_name.lower():
                    lines.append(f"request->{field.field_name} = \"/path/to/file\";")
                else:
                    lines.append(f"request->{field.field_name} = \"example_value\";")
            elif field.type_name in ['float32', 'float64']:
                lines.append(f"request->{field.field_name} = 0.0;")
            elif field.type_name in ['int32', 'uint32', 'int16', 'uint16', 'int64', 'uint64']:
                lines.append(f"request->{field.field_name} = 0;")
            elif field.type_name == 'bool':
                lines.append(f"request->{field.field_name} = true;")
            elif field.is_array:
                lines.append(f"// Fill request->{field.field_name} array as needed")
            else:
                lines.append(f"// Set request->{field.field_name} as needed")
        
        if not lines:
            lines.append("// No request parameters needed")
        
        return "\n        ".join(lines)
    
    def _to_snake_case(self, name: str) -> str:
        """Convert CamelCase to snake_case."""
        result = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
        return re.sub('([a-z0-9])([A-Z])', r'\1_\2', result).lower()
    
    def _load_field_descriptions(self) -> Dict[str, str]:
        """Load common field descriptions for Nav2 services."""
        return {
            # Request fields
            'pose': 'Target pose for the service operation',
            'poses': 'Array of poses to process',
            'path': 'Navigation path to validate or process',
            'map_url': 'URL or file path to map file',
            'filepath': 'File path for loading or saving data',
            'graph_filepath': 'Path to route graph file',
            'reset_distance': 'Distance parameter for clearing operations',
            'max_cost': 'Maximum allowable cost value',
            'consider_unknown_as_obstacle': 'Whether to treat unknown cells as obstacles',
            'use_footprint': 'Whether to use robot footprint for calculations',
            'command': 'Lifecycle command to execute',
            'closed_edges': 'Array of edge IDs to close',
            'opened_edges': 'Array of edge IDs to open',
            'adjust_edges': 'Array of edges with modified costs',
            'specs': 'Specifications for the requested data',
            
            # Response fields
            'success': 'Whether the operation completed successfully',
            'result': 'Result code or status of the operation',
            'response': 'Response message or data',
            'map': 'Map data as occupancy grid',
            'costs': 'Array of cost values',
            'is_valid': 'Whether the validation passed',
            'invalid_pose_indices': 'Indices of poses that failed validation',
            
            # Map service specific
            'map_topic': 'Topic name for the map to save',
            'image_format': 'Image format for saved map (pgm, png, bmp)',
            'map_mode': 'Map saving mode (trinary, scale, raw)',
            'free_thresh': 'Free space threshold value',
            'occupied_thresh': 'Occupied space threshold value',
            
            # Common fields
            'header': 'Standard ROS header with timestamp and frame information',
            'frame_id': 'Frame of reference for the data',
            'stamp': 'Timestamp for the operation'
        }
    
    def _get_field_description(self, field: ServiceField) -> str:
        """Get description for a field, using mapping if no comment exists."""
        if field.comment and field.comment.strip():
            return field.comment.strip()
        
        # Try to find a description based on field name
        if field.field_name in self.field_descriptions:
            return self.field_descriptions[field.field_name]
        
        # Try based on field type patterns
        if 'Header' in field.type_name:
            return 'Standard ROS header with timestamp and frame information'
        elif 'PoseStamped' in field.type_name:
            return 'Pose with header information (frame_id and timestamp)'
        elif 'PoseWithCovarianceStamped' in field.type_name:
            return 'Pose with uncertainty covariance and header information'
        elif 'Path' in field.type_name:
            return 'Navigation path with sequence of poses'
        elif 'OccupancyGrid' in field.type_name:
            return 'Occupancy grid map data'
        elif field.type_name in ['string', 'std_msgs/String']:
            return 'String value or identifier'
        elif field.type_name in ['float32', 'float64']:
            return 'Floating point numerical value'
        elif field.type_name in ['int16', 'int32', 'int64', 'uint16', 'uint32', 'uint64']:
            return 'Integer numerical value'
        elif field.type_name == 'bool':
            return 'Boolean true/false value'
        elif field.is_array:
            return f'Array of {field.base_type} values'
        
        return "Service parameter - see Nav2 documentation for specific usage details"

def main():
    parser = argparse.ArgumentParser(description='Generate Nav2 Service API documentation')
    parser.add_argument('--distribution', required=True, help='ROS 2 distribution name')
    parser.add_argument('--source-dir', required=True, help='Path to Nav2 source directory')
    parser.add_argument('--output-dir', required=True, help='Output directory for generated docs')
    
    args = parser.parse_args()
    
    source_dir = Path(args.source_dir)
    output_dir = Path(args.output_dir)
    
    if not source_dir.exists():
        print(f"Error: Source directory {source_dir} does not exist")
        sys.exit(1)
    
    print(f"Generating service documentation for {args.distribution}")
    print(f"Source directory: {source_dir}")
    print(f"Output directory: {output_dir}")
    
    # Parse services
    parser_obj = ServiceParser(source_dir)
    service_files = parser_obj.find_service_files()
    
    print(f"Found {len(service_files)} service files")
    
    services = []
    seen_services = set()
    for service_file in service_files:
        service = parser_obj.parse_service_file(service_file)
        if service:
            if service.name not in seen_services:
                services.append(service)
                seen_services.add(service.name)
                print(f"  Parsed: {service.name} ({service.category})")
            else:
                print(f"  Skipped: {service.name} (duplicate)")
    
    if not services:
        print("No services found to document")
        return
    
    # Generate documentation
    generator = ServiceDocGenerator(args.distribution, output_dir)
    generator.generate_docs(services)
    
    print(f"Generated documentation for {len(services)} services")
    print(f"Output written to: {output_dir}")

if __name__ == '__main__':
    main()