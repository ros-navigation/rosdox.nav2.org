#!/usr/bin/env python3
"""
Generate ROS Message API documentation for Nav2.
Parses .msg files and generates Jekyll-compatible documentation.
"""

import os
import sys
import json
import argparse
from pathlib import Path
import re
from datetime import datetime
from typing import Dict, List, Any, Optional

class MessageField:
    """Represents a field in a message."""
    def __init__(self, type_name: str, field_name: str, comment: str = "", default_value: str = ""):
        self.type_name = type_name
        self.field_name = field_name
        self.comment = comment.strip()
        self.default_value = default_value.strip()
        self.is_array = "[]" in type_name
        self.base_type = type_name.replace("[]", "")

class MessageDefinition:
    """Represents a complete .msg file."""
    def __init__(self, name: str, package: str, file_path: str):
        self.name = name
        self.package = package
        self.file_path = file_path
        self.fields: List[MessageField] = []
        self.description = self._get_description()
        self.category = self._determine_category()

    def _get_description(self) -> str:
        """Get appropriate description for the message based on its name."""
        descriptions = {
            # Costmap Messages
            'Costmap': 'Occupancy grid representation for navigation planning with cost values',
            'CostmapMetaData': 'Metadata information for costmap including dimensions and resolution',
            'CostmapUpdate': 'Update message containing changed regions of a costmap',
            'CostmapFilterInfo': 'Filter information for costmap layers and processing',
            'VoxelGrid': '3D voxel grid representation for obstacle detection and mapping',
            
            # Behavior Tree Messages
            'BehaviorTreeLog': 'Logging information from behavior tree execution',
            'BehaviorTreeStatusChange': 'Status change events from behavior tree nodes',
            
            # Particle Filter Messages  
            'Particle': 'Single particle hypothesis with pose and weight for particle filter localization',
            'ParticleCloud': 'Collection of particles representing pose distribution in particle filter',
            
            # Route Planning Messages
            'Route': 'Complete route plan with waypoints and metadata',
            'RouteNode': 'Individual waypoint node in a route graph',
            'RouteEdge': 'Connection between route nodes with traversal cost',
            'EdgeCost': 'Cost information for route graph edges',
            
            # Collision Detection Messages
            'CollisionDetectorState': 'State information from collision detection systems',
            'CollisionMonitorState': 'Monitoring state for collision avoidance systems',
            
            # Path Planning Messages
            'Trajectory': 'Time-parameterized path with poses and velocities',
            'TrajectoryPoint': 'Single point in a trajectory with pose, velocity, and timing',
            
            # Speed Regulation Messages
            'SpeedLimit': 'Speed limit information for navigation areas',
            
            # Waypoint Status Messages
            'WaypointStatus': 'Status information for waypoint navigation tasks'
        }
        
        return descriptions.get(self.name, 'Nav2 message for robotic navigation and behavior control')

    def _determine_category(self) -> str:
        """Determine message category based on name."""
        name_lower = self.name.lower()
        
        # Costmap Messages
        if any(keyword in name_lower for keyword in ['costmap', 'voxel']):
            return 'costmap'
        # Behavior Tree Messages
        elif 'behavior' in name_lower or 'tree' in name_lower:
            return 'behavior_tree'
        # Localization Messages
        elif any(keyword in name_lower for keyword in ['particle', 'pose']):
            return 'localization'
        # Route Planning Messages
        elif any(keyword in name_lower for keyword in ['route', 'edge', 'node']):
            return 'routing'
        # Collision Detection Messages
        elif 'collision' in name_lower:
            return 'collision'
        # Path Planning Messages
        elif any(keyword in name_lower for keyword in ['trajectory', 'path']):
            return 'planning'
        # Speed Control Messages
        elif 'speed' in name_lower:
            return 'control'
        # Waypoint Messages
        elif 'waypoint' in name_lower:
            return 'waypoint'
        # Fallback for any other messages
        else:
            return 'other'

class MessageParser:
    """Parser for ROS .msg files."""
    
    def __init__(self, source_dir: Path):
        self.source_dir = source_dir
        
    def find_message_files(self) -> List[Path]:
        """Find all .msg files in the source directory."""
        message_files = []
        
        # Look in nav2_msgs/msg directory
        nav2_msgs_dir = self.source_dir / "nav2_msgs" / "msg"
        if nav2_msgs_dir.exists():
            message_files.extend(nav2_msgs_dir.glob("*.msg"))
            
        # Also look for message files in other nav2 packages
        for pkg_dir in self.source_dir.glob("nav2_*"):
            if pkg_dir.is_dir():
                msg_dir = pkg_dir / "msg"
                if msg_dir.exists():
                    message_files.extend(msg_dir.glob("*.msg"))
        
        return message_files
    
    def parse_message_file(self, message_file: Path) -> Optional[MessageDefinition]:
        """Parse a single .msg file."""
        try:
            with open(message_file, 'r') as f:
                content = f.read()
            
            # Extract package name from path
            package = self._get_package_name(message_file)
            message_name = message_file.stem
            
            message_def = MessageDefinition(message_name, package, str(message_file))
            
            # Parse message content
            message_def.fields = self._parse_message_content(content)
            
            # Use predefined description if available, otherwise extract from comments
            extracted_desc = self._extract_description(content)
            if not message_def.description or message_def.description == 'Nav2 message for robotic navigation and behavior control':
                message_def.description = extracted_desc if extracted_desc else message_def.description
            
            return message_def
            
        except Exception as e:
            print(f"Error parsing {message_file}: {e}")
            return None
    
    def _get_package_name(self, message_file: Path) -> str:
        """Extract package name from file path."""
        # Navigate up to find the package directory
        current = message_file.parent
        while current != self.source_dir:
            if (current / "package.xml").exists():
                return current.name
            current = current.parent
        return "nav2_msgs"  # default
    
    def _parse_message_content(self, content: str) -> List[MessageField]:
        """Parse message content and extract fields."""
        fields = []
        lines = content.strip().split('\n')
        
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
                    fields.append(field)
                current_comment = ""
        
        return fields
    
    def _parse_field_line(self, line: str, comment: str = "") -> Optional[MessageField]:
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
                        
                return MessageField(type_name, field_name, comment, default_value)
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
                        
                return MessageField(type_name, field_name, comment)
        
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

class MessageDocGenerator:
    """Generates Jekyll documentation from parsed messages."""
    
    def __init__(self, distribution: str, output_dir: Path):
        self.distribution = distribution
        self.output_dir = output_dir
        self.field_descriptions = self._load_field_descriptions()
        
    def generate_docs(self, messages: List[MessageDefinition]):
        """Generate all documentation files."""
        # Create output directories
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Group messages by category
        categories = self._group_by_category(messages)
        
        # Generate overview page
        self._generate_overview(messages, categories)
        
        # Generate individual message pages
        for message in messages:
            self._generate_message_page(message)
    
    def _group_by_category(self, messages: List[MessageDefinition]) -> Dict[str, List[MessageDefinition]]:
        """Group messages by category."""
        categories = {}
        for message in messages:
            if message.category not in categories:
                categories[message.category] = []
            categories[message.category].append(message)
        return categories
    
    def _generate_overview(self, messages: List[MessageDefinition], categories: Dict[str, List[MessageDefinition]]):
        """Generate the main overview page."""
        content = f"""---
layout: default
title: "Nav2 Message APIs - {self.distribution.title()}"
permalink: /{self.distribution}/msgs/
---

# Nav2 Message APIs - {self.distribution.title()}

This page documents the ROS Message APIs available in Nav2 for the {self.distribution} distribution. These messages provide data structures for navigation tasks, sensor data, and system status.

## Available Messages ({len(messages)} total)

"""
        
        # Add category sections in desired order
        category_names = {
            'costmap': 'Costmap Messages',
            'behavior_tree': 'Behavior Tree Messages',
            'localization': 'Localization Messages',
            'routing': 'Route Planning Messages',
            'collision': 'Collision Detection Messages',
            'planning': 'Path Planning Messages',
            'control': 'Control Messages',
            'waypoint': 'Waypoint Messages',
            'other': 'Other Messages'
        }
        
        # Define the desired order
        category_order = ['costmap', 'behavior_tree', 'localization', 'routing', 'collision', 'planning', 'control', 'waypoint', 'other']
        
        # Process categories in the specified order
        for category in category_order:
            if category in categories:
                category_messages = sorted(categories[category], key=lambda x: x.name)
                display_name = category_names.get(category, category.title() + ' Messages')
                content += f"""
### {display_name}

<div class="message-grid">
"""
                for message in category_messages:
                    content += f"""  <div class="message-card">
    <h4><a href="/msgs/{self.distribution}/{message.name.lower()}.html">{message.name}</a></h4>
    <p class="message-package">Package: {message.package}</p>
    <p class="message-description">{message.description or 'No description available.'}</p>
  </div>
"""
                content += "</div>\n"
        
        content += f"""

## Related Documentation

- [Nav2 Action APIs](/{self.distribution}/actions/index.html)
- [Nav2 Service APIs](/{self.distribution}/srvs/index.html)
- [Nav2 C++ API Documentation](/{self.distribution}/html/index.html)
- [Nav2 Official Documentation](https://nav2.org/)

*Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S UTC')}*"""
        
        # Write overview file to the correct location
        overview_dir = self.output_dir / self.distribution / "msgs"
        overview_dir.mkdir(parents=True, exist_ok=True)
        
        with open(overview_dir / "index.md", 'w') as f:
            f.write(content)
    
    def _generate_message_page(self, message: MessageDefinition):
        """Generate documentation page for a single message."""
        content = f"""---
layout: default
title: "{message.name} Message"
permalink: /msgs/{self.distribution}/{message.name.lower()}.html
---

# {message.name} Message

**Package:** `{message.package}`  
**Category:** {message.category.replace('_', ' ').title()} Messages

{message.description or "No description available."}

## Message Definition

"""
        
        # Generate message table
        content += self._generate_message_table(message)
        
        # Usage examples (basic examples for common patterns)
        python_example, cpp_example = self._generate_usage_examples(message)
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

## Related Messages

"""
        
        # Add links to related messages in same category
        content += f"- [All {message.category.replace('_', ' ').title()} Messages](/{self.distribution}/msgs/index.html#{message.category.replace('_', '-')}-messages)\n"
        content += f"- [Message API Overview](/{self.distribution}/msgs/index.html)\n"
        content += f"- [Nav2 C++ API Documentation](/{self.distribution}/html/index.html)\n"
        
        # Write message file
        message_dir = self.output_dir / "msgs" / self.distribution
        message_dir.mkdir(parents=True, exist_ok=True)
        
        message_file = message_dir / f"{message.name.lower()}.md"
        with open(message_file, 'w') as f:
            f.write(content)
    
    def _generate_message_table(self, message: MessageDefinition) -> str:
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
    
    def _generate_usage_examples(self, message: MessageDefinition) -> tuple:
        """Generate basic Python and C++ usage examples."""
        message_name = message.name
        package = message.package
        
        # Generate Python example
        python_example = f"""import rclpy
from rclpy.node import Node
from {package}.msg import {message_name}

class {message_name}Publisher(Node):
    def __init__(self):
        super().__init__('{message_name.lower()}_publisher')
        self.publisher = self.create_publisher({message_name}, '{message_name.lower()}', 10)
        
    def publish_message(self):
        msg = {message_name}()
        {self._get_python_field_population(message)}
        self.publisher.publish(msg)"""
        
        # Generate C++ example
        cpp_example = f"""#include "rclcpp/rclcpp.hpp"
#include "{package}/msg/{self._to_snake_case(message_name)}.hpp"

class {message_name}Publisher : public rclcpp::Node
{{
public:
    {message_name}Publisher() : Node("{message_name.lower()}_publisher")
    {{
        publisher_ = create_publisher<{package}::msg::{message_name}>("{message_name.lower()}", 10);
    }}

    void publish_message()
    {{
        auto msg = {package}::msg::{message_name}();
        {self._get_cpp_field_population(message)}
        publisher_->publish(msg);
    }}

private:
    rclcpp::Publisher<{package}::msg::{message_name}>::SharedPtr publisher_;
}};"""
        
        return python_example, cpp_example
    
    def _get_python_field_population(self, message: MessageDefinition) -> str:
        """Get Python code to populate message fields."""
        lines = []
        for field in message.fields:
            if field.default_value:
                continue  # Skip constants
            
            if 'Header' in field.type_name:
                lines.extend([
                    f"msg.{field.field_name}.frame_id = 'map'",
                    f"msg.{field.field_name}.stamp = self.get_clock().now().to_msg()"
                ])
            elif field.type_name in ['string', 'std_msgs/String']:
                lines.append(f"msg.{field.field_name} = 'example_value'")
            elif field.type_name in ['float32', 'float64']:
                lines.append(f"msg.{field.field_name} = 0.0")
            elif field.type_name in ['int32', 'uint32', 'int16', 'uint16', 'int64', 'uint64']:
                lines.append(f"msg.{field.field_name} = 0")
            elif field.type_name == 'bool':
                lines.append(f"msg.{field.field_name} = True")
            elif field.is_array:
                lines.append(f"msg.{field.field_name} = []  # Fill array as needed")
            else:
                lines.append(f"# Set msg.{field.field_name} as needed")
        
        if not lines:
            lines.append("# Set message fields as needed")
        
        return "\n        ".join(lines)
    
    def _get_cpp_field_population(self, message: MessageDefinition) -> str:
        """Get C++ code to populate message fields."""
        lines = []
        for field in message.fields:
            if field.default_value:
                continue  # Skip constants
            
            if 'Header' in field.type_name:
                lines.extend([
                    f"msg.{field.field_name}.frame_id = \"map\";",
                    f"msg.{field.field_name}.stamp = this->now();"
                ])
            elif field.type_name in ['string', 'std_msgs/String']:
                lines.append(f"msg.{field.field_name} = \"example_value\";")
            elif field.type_name in ['float32', 'float64']:
                lines.append(f"msg.{field.field_name} = 0.0;")
            elif field.type_name in ['int32', 'uint32', 'int16', 'uint16', 'int64', 'uint64']:
                lines.append(f"msg.{field.field_name} = 0;")
            elif field.type_name == 'bool':
                lines.append(f"msg.{field.field_name} = true;")
            elif field.is_array:
                lines.append(f"// Fill msg.{field.field_name} array as needed")
            else:
                lines.append(f"// Set msg.{field.field_name} as needed")
        
        if not lines:
            lines.append("// Set message fields as needed")
        
        return "\n        ".join(lines)
    
    def _to_snake_case(self, name: str) -> str:
        """Convert CamelCase to snake_case."""
        result = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
        return re.sub('([a-z0-9])([A-Z])', r'\1_\2', result).lower()
    
    def _load_field_descriptions(self) -> Dict[str, str]:
        """Load common field descriptions for Nav2 messages."""
        return {
            # Common fields
            'header': 'Standard ROS header with timestamp and frame information',
            'stamp': 'Timestamp when the message was created',
            'frame_id': 'Frame of reference for the message data',
            
            # Costmap fields
            'data': 'Occupancy grid data as a flat array of cost values',
            'info': 'Metadata about the costmap including dimensions and resolution',
            'width': 'Width of the costmap in cells',
            'height': 'Height of the costmap in cells', 
            'resolution': 'Meters per cell resolution of the costmap',
            'origin': 'Real-world pose of the cell at (0,0) in the costmap',
            'costs': 'Array of cost values for specified poses',
            
            # Particle filter fields
            'pose': 'Pose hypothesis for the particle',
            'weight': 'Probability weight of this particle hypothesis',
            'particles': 'Array of particles in the particle cloud',
            
            # Route planning fields
            'route': 'Complete route with waypoints and metadata',
            'nodes': 'Array of waypoint nodes in the route',
            'edges': 'Array of connections between route nodes',
            'node_id': 'Unique identifier for the route node',
            'edge_id': 'Unique identifier for the route edge',
            'start_node': 'Starting node ID for the edge',
            'end_node': 'Ending node ID for the edge',
            'edge_cost': 'Cost to traverse this edge',
            
            # Trajectory fields
            'poses': 'Array of poses along the trajectory',
            'velocities': 'Array of velocities corresponding to trajectory poses',
            'time_from_start': 'Time offset from trajectory start for this point',
            
            # Status and state fields
            'status': 'Current status or state value',
            'state': 'Current operational state',
            'success': 'Boolean indicating successful completion',
            'error_code': 'Numeric error code if operation failed',
            'error_msg': 'Human-readable error message',
            
            # Speed and control fields
            'speed': 'Speed limit value in meters per second',
            'max_speed': 'Maximum allowed speed',
            'min_speed': 'Minimum required speed',
            
            # Common constant fields
            'NONE': 'No error or default state value',
            'SUCCESS': 'Operation completed successfully',
            'FAILURE': 'Operation failed',
            'UNKNOWN': 'Unknown or uninitialized state',
            
            # Waypoint fields
            'waypoint_index': 'Index of the current waypoint in sequence',
            'waypoint_pose': 'Pose of the waypoint',
            'is_visited': 'Whether this waypoint has been reached',
            'visit_time': 'Timestamp when waypoint was reached'
        }
    
    def _get_field_description(self, field: MessageField) -> str:
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
        elif 'Duration' in field.type_name:
            return 'Duration or time interval value'
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
        
        return "Message field - see Nav2 documentation for specific usage details"

def main():
    parser = argparse.ArgumentParser(description='Generate Nav2 Message API documentation')
    parser.add_argument('--distribution', required=True, help='ROS 2 distribution name')
    parser.add_argument('--source-dir', required=True, help='Path to Nav2 source directory')
    parser.add_argument('--output-dir', required=True, help='Output directory for generated docs')
    
    args = parser.parse_args()
    
    source_dir = Path(args.source_dir)
    output_dir = Path(args.output_dir)
    
    if not source_dir.exists():
        print(f"Error: Source directory {source_dir} does not exist")
        sys.exit(1)
    
    print(f"Generating message documentation for {args.distribution}")
    print(f"Source directory: {source_dir}")
    print(f"Output directory: {output_dir}")
    
    # Parse messages
    parser_obj = MessageParser(source_dir)
    message_files = parser_obj.find_message_files()
    
    print(f"Found {len(message_files)} message files")
    
    messages = []
    seen_messages = set()
    for message_file in message_files:
        message = parser_obj.parse_message_file(message_file)
        if message:
            if message.name not in seen_messages:
                messages.append(message)
                seen_messages.add(message.name)
                print(f"  Parsed: {message.name} ({message.category})")
            else:
                print(f"  Skipped: {message.name} (duplicate)")
    
    if not messages:
        print("No messages found to document")
        return
    
    # Generate documentation
    generator = MessageDocGenerator(args.distribution, output_dir)
    generator.generate_docs(messages)
    
    print(f"Generated documentation for {len(messages)} messages")
    print(f"Output written to: {output_dir}")

if __name__ == '__main__':
    main()