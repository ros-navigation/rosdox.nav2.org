#!/usr/bin/env python3
"""
Generate ROS Action API documentation for Nav2.
Parses .action files and generates Jekyll-compatible documentation.
"""

import os
import sys
import json
import argparse
from pathlib import Path
import re
from datetime import datetime
from typing import Dict, List, Any, Optional

class ActionField:
    """Represents a field in an action message."""
    def __init__(self, type_name: str, field_name: str, comment: str = ""):
        self.type_name = type_name
        self.field_name = field_name
        self.comment = comment.strip()
        self.is_array = "[]" in type_name
        self.base_type = type_name.replace("[]", "")

class ActionMessage:
    """Represents a Goal, Result, or Feedback message."""
    def __init__(self, name: str):
        self.name = name
        self.fields: List[ActionField] = []
        self.description = ""

class ActionDefinition:
    """Represents a complete .action file."""
    def __init__(self, name: str, package: str, file_path: str):
        self.name = name
        self.package = package
        self.file_path = file_path
        self.goal = ActionMessage("Goal")
        self.result = ActionMessage("Result")
        self.feedback = ActionMessage("Feedback")
        self.description = self._get_description()
        self.category = self._determine_category()

    def _get_description(self) -> str:
        """Get appropriate description for the action based on its name."""
        descriptions = {
            # Navigation Actions
            'NavigateToPose': 'Navigate robot to a specific pose with obstacle avoidance and recovery behaviors',
            'NavigateThroughPoses': 'Navigate robot through a sequence of poses in order',
            'FollowWaypoints': 'Navigate robot through a series of waypoints with optional task execution at each point',
            'FollowGPSWaypoints': 'Navigate robot through GPS-based waypoints for outdoor navigation',
            
            # Planning Actions
            'ComputePathToPose': 'Compute an optimal path from current position to a target pose',
            'ComputePathThroughPoses': 'Compute an optimal path connecting multiple poses in sequence',
            'ComputeRoute': 'Compute a high-level route between waypoints using graph search',
            'ComputeAndTrackRoute': 'Compute and actively track a route with dynamic replanning',
            'SmoothPath': 'Generate a smoother, kinematically feasible path from a discrete path',
            
            # Controller Actions
            'FollowPath': 'Execute path following using a specified controller with progress monitoring',
            
            # Behavior Actions
            'Spin': 'Rotate robot in place to a target yaw angle with collision checking',
            'BackUp': 'Move robot backwards a specified distance with obstacle detection',
            'Wait': 'Pause robot operation for a specified duration or until condition is met',
            'DriveOnHeading': 'Drive robot forward in a specific direction for a given distance',
            'AssistedTeleop': 'Provide assisted teleoperation with collision avoidance and safety checks',
            
            # AutoDocking Actions
            'DockRobot': 'Autonomously dock robot to a charging station or docking platform',
            'UndockRobot': 'Autonomously undock robot from a charging station or docking platform'
        }
        
        return descriptions.get(self.name, 'Nav2 action for robotic navigation and behavior control')

    def _determine_category(self) -> str:
        """Determine action category based on name."""
        name_lower = self.name.lower()
        
        # Navigation Actions
        if self.name in ['NavigateToPose', 'NavigateThroughPoses', 'FollowWaypoints', 'FollowGPSWaypoints']:
            return 'navigation'
        # Controller Actions
        elif self.name in ['FollowPath']:
            return 'controller'
        # Planning Actions  
        elif self.name in ['SmoothPath', 'ComputeRoute', 'ComputeAndTrackRoute', 'ComputePathThroughPoses', 'ComputePathToPose']:
            return 'planning'
        # Behaviors (formerly Recovery)
        elif self.name in ['Spin', 'BackUp', 'Wait', 'DriveOnHeading', 'AssistedTeleop']:
            return 'behaviors'
        # AutoDocking
        elif self.name in ['DockRobot', 'UndockRobot']:
            return 'autodocking'
        # Fallback for any other actions
        else:
            return 'other'

class ActionParser:
    """Parser for ROS .action files."""
    
    def __init__(self, source_dir: Path):
        self.source_dir = source_dir
        
    def find_action_files(self) -> List[Path]:
        """Find all .action files in the source directory."""
        action_files = []
        
        # Look in nav2_msgs/action directory
        nav2_msgs_dir = self.source_dir / "nav2_msgs" / "action"
        if nav2_msgs_dir.exists():
            action_files.extend(nav2_msgs_dir.glob("*.action"))
            
        # Also look for action files in other nav2 packages
        for pkg_dir in self.source_dir.glob("nav2_*"):
            if pkg_dir.is_dir():
                action_dir = pkg_dir / "action"
                if action_dir.exists():
                    action_files.extend(action_dir.glob("*.action"))
        
        return action_files
    
    def parse_action_file(self, action_file: Path) -> Optional[ActionDefinition]:
        """Parse a single .action file."""
        try:
            with open(action_file, 'r') as f:
                content = f.read()
            
            # Extract package name from path
            package = self._get_package_name(action_file)
            action_name = action_file.stem
            
            action_def = ActionDefinition(action_name, package, str(action_file))
            
            # Split content into Goal, Result, and Feedback sections
            sections = content.split('---')
            if len(sections) >= 3:
                action_def.goal = self._parse_message_section(sections[0], "Goal")
                action_def.result = self._parse_message_section(sections[1], "Result")
                action_def.feedback = self._parse_message_section(sections[2], "Feedback")
            elif len(sections) == 2:
                action_def.goal = self._parse_message_section(sections[0], "Goal")
                action_def.result = self._parse_message_section(sections[1], "Result")
            elif len(sections) == 1:
                action_def.goal = self._parse_message_section(sections[0], "Goal")
            
            # Use predefined description if available, otherwise extract from comments
            extracted_desc = self._extract_description(content)
            if not action_def.description or action_def.description == 'Nav2 action for robotic navigation and behavior control':
                action_def.description = extracted_desc if extracted_desc else action_def.description
            
            return action_def
            
        except Exception as e:
            print(f"Error parsing {action_file}: {e}")
            return None
    
    def _get_package_name(self, action_file: Path) -> str:
        """Extract package name from file path."""
        # Navigate up to find the package directory
        current = action_file.parent
        while current != self.source_dir:
            if (current / "package.xml").exists():
                return current.name
            current = current.parent
        return "nav2_msgs"  # default
    
    def _parse_message_section(self, section: str, section_name: str) -> ActionMessage:
        """Parse a message section (Goal, Result, or Feedback)."""
        message = ActionMessage(section_name)
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
    
    def _parse_field_line(self, line: str, comment: str = "") -> Optional[ActionField]:
        """Parse a single field definition line."""
        # Match patterns like "string target_frame" or "float64[] path"
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
                    
            return ActionField(type_name, field_name, comment)
        
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

class ActionDocGenerator:
    """Generates Jekyll documentation from parsed actions."""
    
    def __init__(self, distribution: str, output_dir: Path):
        self.distribution = distribution
        self.output_dir = output_dir
        self.field_descriptions = self._load_field_descriptions()
        
    def generate_docs(self, actions: List[ActionDefinition]):
        """Generate all documentation files."""
        # Create output directories
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # Group actions by category
        categories = self._group_by_category(actions)
        
        # Generate overview page
        self._generate_overview(actions, categories)
        
        # Generate individual action pages
        for action in actions:
            self._generate_action_page(action)
        
        # Generate category index pages
        for category, category_actions in categories.items():
            self._generate_category_page(category, category_actions)
    
    def _group_by_category(self, actions: List[ActionDefinition]) -> Dict[str, List[ActionDefinition]]:
        """Group actions by category."""
        categories = {}
        for action in actions:
            if action.category not in categories:
                categories[action.category] = []
            categories[action.category].append(action)
        return categories
    
    def _generate_overview(self, actions: List[ActionDefinition], categories: Dict[str, List[ActionDefinition]]):
        """Generate the main overview page."""
        content = f"""---
layout: default
title: "Nav2 Action APIs - {self.distribution.title()}"
permalink: /{self.distribution}/actions/
---

# Nav2 Action APIs - {self.distribution.title()}

This page documents the ROS Action APIs available in Nav2 for the {self.distribution} distribution. These actions provide high-level interfaces for navigation tasks, planning, and recovery behaviors.

## Available Actions ({len(actions)} total)

"""
        
        # Add category sections in desired order
        category_names = {
            'navigation': 'Navigation Actions',
            'planning': 'Planning Actions',
            'controller': 'Controller Actions',
            'behaviors': 'Behaviors',
            'autodocking': 'AutoDocking',
            'other': 'Other Actions'
        }
        
        # Define the desired order
        category_order = ['navigation', 'planning', 'controller', 'behaviors', 'autodocking', 'other']
        
        # Process categories in the specified order
        for category in category_order:
            if category in categories:
                category_actions = sorted(categories[category], key=lambda x: x.name)
                display_name = category_names.get(category, category.title() + ' Actions')
                content += f"""
### {display_name}

<div class="action-grid">
"""
                for action in category_actions:
                    content += f"""  <div class="action-card">
    <h4><a href="/actions/{self.distribution}/{action.name.lower()}.html">{action.name}</a></h4>
    <p class="action-package">Package: {action.package}</p>
    <p class="action-description">{action.description or 'No description available.'}</p>
  </div>
"""
                content += "</div>\n"
        
        content += f"""

## Related Documentation

- [Nav2 C++ API Documentation](/{self.distribution}/html/index.html)
- [Nav2 Official Documentation](https://nav2.org/)
- [ROS 2 Actions Documentation](https://docs.ros.org/en/{self.distribution}/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)

*Generated on {datetime.now().strftime('%Y-%m-%d %H:%M:%S UTC')}*"""
        
        # Write overview file to the correct location
        overview_dir = self.output_dir / self.distribution / "actions"
        overview_dir.mkdir(parents=True, exist_ok=True)
        
        with open(overview_dir / "index.md", 'w') as f:
            f.write(content)
    
    def _generate_action_page(self, action: ActionDefinition):
        """Generate documentation page for a single action."""
        content = f"""---
layout: default
title: "{action.name} Action"
permalink: /actions/{self.distribution}/{action.name.lower()}.html
---

# {action.name} Action

**Package:** `{action.package}`  
**Category:** {action.category.title()}

{action.description or "No description available."}

## Message Definitions

"""
        
        # Goal section
        content += "### Goal Message\n\n"
        content += self._generate_message_table(action.goal)
        
        # Result section  
        content += "\n### Result Message\n\n"
        content += self._generate_message_table(action.result)
        
        # Feedback section
        content += "\n### Feedback Message\n\n"
        content += self._generate_message_table(action.feedback)
        
        # Usage examples
        python_example, cpp_example = self._generate_realistic_examples(action)
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

## Related Actions

"""
        
        # Add links to related actions in same category
        content += f"- [All {action.category.title()} Actions](/{self.distribution}/actions/index.html#{action.category})\n"
        content += f"- [Action API Overview](/{self.distribution}/actions/index.html)\n"
        content += f"- [Nav2 C++ API Documentation](/{self.distribution}/html/index.html)\n"
        
        # Write action file
        action_dir = self.output_dir / "actions" / self.distribution
        action_dir.mkdir(parents=True, exist_ok=True)
        
        action_file = action_dir / f"{action.name.lower()}.md"
        with open(action_file, 'w') as f:
            f.write(content)
    
    def _generate_message_table(self, message: ActionMessage) -> str:
        """Generate markdown table for message fields."""
        if not message.fields:
            return "No fields defined.\n"
        
        table = "| Field | Type | Description |\n"
        table += "|-------|------|-------------|\n"
        
        for field in message.fields:
            description = self._get_field_description(field)
            table += f"| `{field.field_name}` | `{field.type_name}` | {description} |\n"
        
        return table + "\n"
    
    def _generate_category_page(self, category: str, actions: List[ActionDefinition]):
        """Generate a category overview page."""
        category_title = category.replace('_', ' ').title()
        
        content = f"""---
layout: default
title: "{category_title} Actions - {self.distribution.title()}"
permalink: /{self.distribution}/actions/{category}/
---

# {category_title} Actions - {self.distribution.title()}

This page lists all {category_title.lower()} actions available in Nav2 for the {self.distribution} distribution.

## Available Actions ({len(actions)})

"""
        
        for action in actions:
            content += f"""
### [{action.name}](/actions/{self.distribution}/{action.name.lower()}.html)

**Package:** `{action.package}`

{action.description or "No description available."}

---
"""
        
        content += f"""

[Back to All Actions](/{self.distribution}/actions/index.html)
"""
        
        # Write category file
        category_dir = self.output_dir / "actions" / self.distribution / category
        category_dir.mkdir(parents=True, exist_ok=True)
        
        with open(category_dir / "index.md", 'w') as f:
            f.write(content)
    
    def _to_snake_case(self, name: str) -> str:
        """Convert CamelCase to snake_case."""
        result = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', name)
        return re.sub('([a-z0-9])([A-Z])', r'\1_\2', result).lower()
    
    def _generate_realistic_examples(self, action: ActionDefinition) -> tuple:
        """Generate realistic Python and C++ examples based on action type."""
        action_name = action.name
        package = action.package
        snake_name = self._to_snake_case(action_name)
        
        # Generate Python example with realistic content
        python_example = f"""import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from {package}.action import {action_name}

class Nav2ActionClient(Node):
    def __init__(self):
        super().__init__('nav2_action_client')
        self.action_client = ActionClient(self, {action_name}, '{snake_name}')
        
    def send_goal(self):
        goal_msg = {action_name}.Goal()
        {self._get_python_goal_population(action_name)}
        
        self.action_client.wait_for_server()
        future = self.action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)
        return future
        
    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f'Received feedback: {{feedback_msg.feedback}}')"""
        
        # Generate C++ example with realistic content
        cpp_example = f"""#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "{package}/action/{snake_name}.hpp"

class Nav2ActionClient : public rclcpp::Node
{{
public:
    using {action_name}Action = {package}::action::{action_name};
    using GoalHandle = rclcpp_action::ClientGoalHandle<{action_name}Action>;

    Nav2ActionClient() : Node("nav2_action_client")
    {{
        action_client_ = rclcpp_action::create_client<{action_name}Action>(
            this, "{snake_name}");
    }}

    void send_goal()
    {{
        auto goal_msg = {action_name}Action::Goal();
        {self._get_cpp_goal_population(action_name)}
        
        action_client_->wait_for_action_server();
        
        auto send_goal_options = rclcpp_action::Client<{action_name}Action>::SendGoalOptions();
        send_goal_options.feedback_callback = 
            std::bind(&Nav2ActionClient::feedback_callback, this, 
                     std::placeholders::_1, std::placeholders::_2);
        
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }}

private:
    rclcpp_action::Client<{action_name}Action>::SharedPtr action_client_;
    
    void feedback_callback(GoalHandle::SharedPtr, 
                          const std::shared_ptr<const {action_name}Action::Feedback> feedback)
    {{
        RCLCPP_INFO(this->get_logger(), "Received feedback");
    }}
}};"""
        
        return python_example, cpp_example
    
    def _get_python_goal_population(self, action_name: str) -> str:
        """Get Python code to populate goal message based on action type."""
        examples = {
            'NavigateToPose': '''goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = 2.0
        goal_msg.pose.pose.position.y = 1.0
        goal_msg.pose.pose.orientation.w = 1.0''',
            
            'NavigateThroughPoses': '''goal_msg.poses = []
        pose1 = geometry_msgs.msg.PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 1.0
        pose1.pose.position.y = 1.0
        pose1.pose.orientation.w = 1.0
        goal_msg.poses.append(pose1)''',
            
            'FollowPath': '''goal_msg.path.header.frame_id = 'map'
        goal_msg.path.header.stamp = self.get_clock().now().to_msg()
        goal_msg.controller_id = 'FollowPath'
        goal_msg.goal_checker_id = 'simple_goal_checker' ''',
        
            'FollowWaypoints': '''waypoint = geometry_msgs.msg.PoseStamped()
        waypoint.header.frame_id = 'map'
        waypoint.pose.position.x = 2.0
        waypoint.pose.position.y = 1.5
        waypoint.pose.orientation.w = 1.0
        goal_msg.poses = [waypoint]''',
            
            'ComputePathToPose': '''goal_msg.goal.header.frame_id = 'map'
        goal_msg.goal.pose.position.x = 3.0
        goal_msg.goal.pose.position.y = 2.0
        goal_msg.goal.pose.orientation.w = 1.0
        goal_msg.planner_id = 'GridBased'
        goal_msg.use_start = False''',
            
            'Spin': '''goal_msg.target_yaw = 1.57
        goal_msg.time_allowance = Duration(seconds=10.0)''',
            
            'BackUp': '''goal_msg.target.x = -0.5
        goal_msg.target.y = 0.0
        goal_msg.target.z = 0.0
        goal_msg.speed = 0.25
        goal_msg.time_allowance = Duration(seconds=10.0)''',
            
            'Wait': '''goal_msg.time = Duration(seconds=5.0)''',
            
            'DockRobot': '''goal_msg.use_dock_id = True
        goal_msg.dock_id = 'dock_01' 
        goal_msg.dock_type = 'nova_carter_dock' 
        goal_msg.max_staging_time = Duration(seconds=30.0)''',
            
            'UndockRobot': '''goal_msg.dock_type = 'nova_carter_dock' ''',
            
            'DriveOnHeading': '''goal_msg.speed = 0.5
        goal_msg.target.x = 2.0
        goal_msg.target.y = 0.0
        goal_msg.target.z = 0.0
        goal_msg.time_allowance = Duration(seconds=10.0)'''
        }
        
        return examples.get(action_name, 
                          f"# Set appropriate fields for {action_name}")
    
    def _get_cpp_goal_population(self, action_name: str) -> str:
        """Get C++ code to populate goal message based on action type."""
        examples = {
            'NavigateToPose': '''goal_msg.pose.header.frame_id = "map";
        goal_msg.pose.header.stamp = this->now();
        goal_msg.pose.pose.position.x = 2.0;
        goal_msg.pose.pose.position.y = 1.0;
        goal_msg.pose.pose.orientation.w = 1.0;''',
            
            'NavigateThroughPoses': '''geometry_msgs::msg::PoseStamped pose1;
        pose1.header.frame_id = "map";
        pose1.pose.position.x = 1.0;
        pose1.pose.position.y = 1.0;
        pose1.pose.orientation.w = 1.0;
        goal_msg.poses.push_back(pose1);''',
            
            'FollowPath': '''goal_msg.path.header.frame_id = "map";
        goal_msg.path.header.stamp = this->now();
        goal_msg.controller_id = "FollowPath";
        goal_msg.goal_checker_id = "simple_goal_checker";''',
        
            'FollowWaypoints': '''geometry_msgs::msg::PoseStamped waypoint;
        waypoint.header.frame_id = "map";
        waypoint.pose.position.x = 2.0;
        waypoint.pose.position.y = 1.5;
        waypoint.pose.orientation.w = 1.0;
        goal_msg.poses.push_back(waypoint);''',
            
            'ComputePathToPose': '''goal_msg.goal.header.frame_id = "map";
        goal_msg.goal.pose.position.x = 3.0;
        goal_msg.goal.pose.position.y = 2.0;
        goal_msg.goal.pose.orientation.w = 1.0;
        goal_msg.planner_id = "GridBased";
        goal_msg.use_start = false;''',
            
            'Spin': '''goal_msg.target_yaw = 1.57;
        goal_msg.time_allowance = rclcpp::Duration::from_seconds(10.0);''',
            
            'BackUp': '''goal_msg.target.x = -0.5;
        goal_msg.target.y = 0.0;
        goal_msg.target.z = 0.0;
        goal_msg.speed = 0.25;
        goal_msg.time_allowance = rclcpp::Duration::from_seconds(10.0);''',
            
            'Wait': '''goal_msg.time = rclcpp::Duration::from_seconds(5.0);''',
            
            'DockRobot': '''goal_msg.use_dock_id = true;
        goal_msg.dock_id = "dock_01";
        goal_msg.dock_type = "nova_carter_dock";
        goal_msg.max_staging_time = rclcpp::Duration::from_seconds(30.0);''',
            
            'UndockRobot': '''goal_msg.dock_type = "nova_carter_dock";''',
            
            'DriveOnHeading': '''goal_msg.speed = 0.5;
        goal_msg.target.x = 2.0;
        goal_msg.target.y = 0.0;
        goal_msg.target.z = 0.0;
        goal_msg.time_allowance = rclcpp::Duration::from_seconds(10.0);'''
        }
        
        return examples.get(action_name, 
                          f"// Set appropriate fields for {action_name}")
    
    def _load_field_descriptions(self) -> Dict[str, str]:
        """Load common field descriptions for Nav2 actions."""
        return {
            # Common pose and geometry fields
            'pose': 'Target pose for navigation in the specified frame',
            'poses': 'Array of poses defining waypoints or path',
            'current_pose': 'Current robot pose during navigation',
            'goal': 'Target goal pose for path planning',
            'start': 'Starting pose for path planning',
            'path': 'Computed navigation path with poses and metadata',
            
            # Navigation parameters
            'behavior_tree': 'Optional behavior tree XML to use for this navigation task',
            'planner_id': 'Name of the planner plugin to use for path planning',
            'controller_id': 'Name of the controller plugin to use for path following',
            'goal_checker_id': 'Name of the goal checker plugin to use',
            'use_start': 'Whether to use the start pose or current robot pose',
            
            # Time and duration fields
            'navigation_time': 'Total time elapsed since navigation started',
            'estimated_time_remaining': 'Estimated time remaining to reach the goal',
            'time_allowance': 'Maximum time allowed for this action to complete',
            'max_staging_time': 'Maximum time to spend in staging before docking',
            'time': 'Duration to wait or time parameter for the action',
            
            # Recovery and status fields
            'number_of_recoveries': 'Number of recovery behaviors executed during navigation',
            'distance_remaining': 'Approximate distance remaining to the goal',
            'result': 'Empty result indicating successful completion',
            
            # Movement parameters
            'target_yaw': 'Target yaw angle to spin to (in radians)',
            'target': 'Target position or velocity vector',
            'speed': 'Speed for movement (m/s)',
            
            # Docking parameters
            'dock_id': 'Unique identifier for the dock to use',
            'dock_type': 'Type or model of the docking station',
            'use_dock_id': 'Whether to use dock_id or auto-detect dock',
            
            # GPS and routing
            'route_end': 'Final destination for route planning',
            'route_start': 'Starting point for route planning (optional)',
            'route': 'Computed route with waypoints and metadata',
            
            # Assisted teleop
            'command': 'Teleop command to be processed and executed safely',
            
            # General result fields
            'error_code': 'Error code indicating the result status',
            'total_elapsed_time': 'Total time taken to complete the action',
            'planning_time': 'Time spent in path planning phase'
        }
    
    def _get_field_description(self, field: ActionField) -> str:
        """Get description for a field, using mapping if no comment exists."""
        if field.comment and field.comment.strip() and field.comment.strip() not in ['goal definition', 'result definition', 'feedback definition']:
            return field.comment
        
        # Try to find a description based on field name
        if field.field_name in self.field_descriptions:
            return self.field_descriptions[field.field_name]
        
        # Try based on field type patterns
        if 'PoseStamped' in field.type_name:
            return 'Pose with header information (frame_id and timestamp)'
        elif 'Duration' in field.type_name:
            return 'Time duration value'
        elif 'Empty' in field.type_name:
            return 'Empty message (no data fields)'
        elif field.type_name in ['string', 'std_msgs/String']:
            return 'Text string parameter'
        elif field.type_name in ['float32', 'float64']:
            return 'Floating point numeric value'
        elif field.type_name in ['int16', 'int32', 'int64', 'uint16', 'uint32', 'uint64']:
            return 'Integer numeric value'
        elif field.type_name == 'bool':
            return 'Boolean true/false flag'
        
        return "Parameter for the action (see Nav2 documentation)"

def main():
    parser = argparse.ArgumentParser(description='Generate Nav2 Action API documentation')
    parser.add_argument('--distribution', required=True, help='ROS 2 distribution name')
    parser.add_argument('--source-dir', required=True, help='Path to Nav2 source directory')
    parser.add_argument('--output-dir', required=True, help='Output directory for generated docs')
    
    args = parser.parse_args()
    
    source_dir = Path(args.source_dir)
    output_dir = Path(args.output_dir)
    
    if not source_dir.exists():
        print(f"Error: Source directory {source_dir} does not exist")
        sys.exit(1)
    
    print(f"Generating action documentation for {args.distribution}")
    print(f"Source directory: {source_dir}")
    print(f"Output directory: {output_dir}")
    
    # Parse actions
    parser_obj = ActionParser(source_dir)
    action_files = parser_obj.find_action_files()
    
    print(f"Found {len(action_files)} action files")
    
    actions = []
    seen_actions = set()
    for action_file in action_files:
        action = parser_obj.parse_action_file(action_file)
        if action and not action.name.startswith('Dummy'):
            if action.name not in seen_actions:
                actions.append(action)
                seen_actions.add(action.name)
                print(f"  Parsed: {action.name} ({action.category})")
            else:
                print(f"  Skipped: {action.name} (duplicate)")
        elif action and action.name.startswith('Dummy'):
            print(f"  Skipped: {action.name} (dummy action)")
    
    if not actions:
        print("No actions found to document")
        return
    
    # Generate documentation
    generator = ActionDocGenerator(args.distribution, output_dir)
    generator.generate_docs(actions)
    
    print(f"Generated documentation for {len(actions)} actions")
    print(f"Output written to: {output_dir}")

if __name__ == '__main__':
    main()