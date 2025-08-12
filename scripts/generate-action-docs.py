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
        goal_msg.time_allowance = Duration(seconds=10.0)''',
        
            'FollowGPSWaypoints': '''from geographic_msgs.msg import GeoPose
        from geometry_msgs.msg import Point
        from geometry_msgs.msg import Quaternion
        
        # Create GPS waypoints
        gps_pose1 = GeoPose()
        gps_pose1.position.latitude = 37.4419    # Example latitude 
        gps_pose1.position.longitude = -122.1430 # Example longitude
        gps_pose1.position.altitude = 0.0
        gps_pose1.orientation.w = 1.0
        
        gps_pose2 = GeoPose()
        gps_pose2.position.latitude = 37.4420
        gps_pose2.position.longitude = -122.1431
        gps_pose2.position.altitude = 0.0
        gps_pose2.orientation.w = 1.0
        
        goal_msg.gps_poses = [gps_pose1, gps_pose2]
        goal_msg.number_of_loops = 1
        goal_msg.goal_index = 0''',
        
            'AssistedTeleop': '''goal_msg.time_allowance = Duration(seconds=60.0)''',
            
            'ComputePathThroughPoses': '''from nav_msgs.msg import Goals
        from geometry_msgs.msg import PoseStamped
        
        # Create goals to connect
        goals = Goals()
        
        pose1 = PoseStamped()
        pose1.header.frame_id = 'map'
        pose1.pose.position.x = 1.0
        pose1.pose.position.y = 1.0
        pose1.pose.orientation.w = 1.0
        
        pose2 = PoseStamped()
        pose2.header.frame_id = 'map'
        pose2.pose.position.x = 3.0
        pose2.pose.position.y = 2.0
        pose2.pose.orientation.w = 1.0
        
        goals.poses = [pose1, pose2]
        goal_msg.goals = goals
        goal_msg.planner_id = 'GridBased'
        goal_msg.use_start = False''',
        
            'ComputeAndTrackRoute': '''from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        
        # Create route as path
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        waypoint1 = PoseStamped()
        waypoint1.header.frame_id = 'map'
        waypoint1.pose.position.x = 1.0
        waypoint1.pose.position.y = 1.0
        waypoint1.pose.orientation.w = 1.0
        
        waypoint2 = PoseStamped()
        waypoint2.header.frame_id = 'map'
        waypoint2.pose.position.x = 5.0
        waypoint2.pose.position.y = 3.0
        waypoint2.pose.orientation.w = 1.0
        
        path.poses = [waypoint1, waypoint2]
        goal_msg.route = path''',
        
            'ComputeRoute': '''from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        
        # Define start point
        start = PoseStamped()
        start.header.frame_id = 'map'
        start.pose.position.x = 0.0
        start.pose.position.y = 0.0
        start.pose.orientation.w = 1.0
        
        # Define goal point
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.pose.position.x = 10.0
        goal.pose.position.y = 5.0
        goal.pose.orientation.w = 1.0
        
        goal_msg.start = start
        goal_msg.goal = goal''',
        
            'SmoothPath': '''from nav_msgs.msg import Path
        from geometry_msgs.msg import PoseStamped
        
        # Create path to smooth
        path = Path()
        path.header.frame_id = 'map'
        path.header.stamp = self.get_clock().now().to_msg()
        
        # Add waypoints to path
        for i in range(5):
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(i)
            pose.pose.position.y = 0.0
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        
        goal_msg.path = path
        goal_msg.smoother_id = 'simple_smoother'
        goal_msg.max_smoothing_duration = Duration(seconds=5.0)'''
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
        goal_msg.time_allowance = rclcpp::Duration::from_seconds(10.0);''',
        
            'FollowGPSWaypoints': '''// Create GPS waypoints
        geographic_msgs::msg::GeoPose gps_pose1, gps_pose2;
        
        gps_pose1.position.latitude = 37.4419;    // Example latitude
        gps_pose1.position.longitude = -122.1430; // Example longitude
        gps_pose1.position.altitude = 0.0;
        gps_pose1.orientation.w = 1.0;
        
        gps_pose2.position.latitude = 37.4420;
        gps_pose2.position.longitude = -122.1431;
        gps_pose2.position.altitude = 0.0;
        gps_pose2.orientation.w = 1.0;
        
        goal_msg.gps_poses = {gps_pose1, gps_pose2};
        goal_msg.number_of_loops = 1;
        goal_msg.goal_index = 0;''',
        
            'AssistedTeleop': '''goal_msg.time_allowance = rclcpp::Duration::from_seconds(60.0);''',
            
            'ComputePathThroughPoses': '''// Create goals to connect
        nav_msgs::msg::Goals goals;
        geometry_msgs::msg::PoseStamped pose1, pose2;
        
        pose1.header.frame_id = "map";
        pose1.pose.position.x = 1.0;
        pose1.pose.position.y = 1.0;
        pose1.pose.orientation.w = 1.0;
        
        pose2.header.frame_id = "map";
        pose2.pose.position.x = 3.0;
        pose2.pose.position.y = 2.0;
        pose2.pose.orientation.w = 1.0;
        
        goals.poses = {pose1, pose2};
        goal_msg.goals = goals;
        goal_msg.planner_id = "GridBased";
        goal_msg.use_start = false;''',
        
            'ComputeAndTrackRoute': '''// Create route as path
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        
        geometry_msgs::msg::PoseStamped waypoint1, waypoint2;
        waypoint1.header.frame_id = "map";
        waypoint1.pose.position.x = 1.0;
        waypoint1.pose.position.y = 1.0;
        waypoint1.pose.orientation.w = 1.0;
        
        waypoint2.header.frame_id = "map";
        waypoint2.pose.position.x = 5.0;
        waypoint2.pose.position.y = 3.0;
        waypoint2.pose.orientation.w = 1.0;
        
        path.poses = {waypoint1, waypoint2};
        goal_msg.route = path;''',
        
            'ComputeRoute': '''// Define start and goal points
        geometry_msgs::msg::PoseStamped start, goal;
        
        start.header.frame_id = "map";
        start.pose.position.x = 0.0;
        start.pose.position.y = 0.0;
        start.pose.orientation.w = 1.0;
        
        goal.header.frame_id = "map";
        goal.pose.position.x = 10.0;
        goal.pose.position.y = 5.0;
        goal.pose.orientation.w = 1.0;
        
        goal_msg.start = start;
        goal_msg.goal = goal;''',
        
            'SmoothPath': '''// Create path to smooth
        nav_msgs::msg::Path path;
        path.header.frame_id = "map";
        path.header.stamp = this->now();
        
        // Add waypoints to path
        for (int i = 0; i < 5; i++) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header.frame_id = "map";
            pose.pose.position.x = static_cast<double>(i);
            pose.pose.position.y = 0.0;
            pose.pose.orientation.w = 1.0;
            path.poses.push_back(pose);
        }
        
        goal_msg.path = path;
        goal_msg.smoother_id = "simple_smoother";
        goal_msg.max_smoothing_duration = rclcpp::Duration::from_seconds(5.0);'''
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
            'behavior_tree': 'Path to custom behavior tree XML file to use for this navigation task. If empty, uses default navigation behavior tree with planning, following, and recovery behaviors',
            'planner_id': 'Name of the specific planning algorithm to use (e.g., "GridBased", "NavfnPlanner"). If empty with single planner, uses default',
            'controller_id': 'Name of the path following controller to use (e.g., "FollowPath", "RegulatedPurePursuit")',
            'goal_checker_id': 'Name of the goal checker plugin to use',
            'use_start': 'Whether to use the provided start pose (true) or current robot position (false) as path planning origin',
            
            # Time and duration fields
            'navigation_time': 'Total time elapsed since navigation started',
            'estimated_time_remaining': 'Estimated time remaining to reach the goal',
            'time_allowance': 'Maximum time limit for completing the action before timing out',
            'max_staging_time': 'Maximum time to spend in staging before docking',
            'time': 'Duration to wait/pause robot motion before continuing',
            
            # Recovery and status fields
            'number_of_recoveries': 'Count of recovery behaviors executed during navigation to overcome obstacles or failures',
            'distance_remaining': 'Approximate distance remaining to the goal',
            'distance_traveled': 'Real-time feedback of distance moved from starting position',
            'result': 'Empty result indicating successful completion',
            
            # Movement parameters
            'target_yaw': 'Target rotation angle in radians to spin (positive=counterclockwise, negative=clockwise)',
            'target': 'Target position or velocity vector',
            'speed': 'Movement speed in meters per second for the specified motion',
            'dist_to_travel': 'Distance in meters to travel in the specified direction',
            'disable_collision_checks': 'Whether to skip obstacle detection during motion (false=check for collisions, true=ignore obstacles)',
            
            # Docking parameters
            'dock_id': 'Identifier name of the docking station from the dock database to autonomously navigate to and dock with',
            'dock_type': 'Type/model of the docking station (e.g., "nova_carter_dock") for proper docking/undocking sequence',
            'use_dock_id': 'Whether to use dock database ID (true) or manually specified dock pose and type (false)',
            'navigate_to_staging_pose': 'Whether to navigate to the staging pose before docking',
            
            # GPS and routing
            'route_end': 'Final destination for route planning',
            'route_start': 'Starting point for route planning (optional)',
            'route': 'Computed route with waypoints and metadata',
            'number_of_loops': 'How many times to repeat the complete waypoint sequence (0=no looping)',
            'goal_index': 'Starting waypoint index in the poses array (default 0 for beginning)',
            
            # Assisted teleop
            'command': 'Teleop command to be processed and executed safely',
            
            # General result and error fields
            'error_code': 'Numeric error code indicating specific failure reason (0=success, various codes for different failure types)',
            'error_msg': 'Human-readable error message describing what went wrong during action execution',
            'total_elapsed_time': 'Total time taken to complete the action',
            'planning_time': 'Time spent in path planning phase',
            
            # Error code constants
            'NONE': 'Success status code indicating the action completed without errors',
            'UNKNOWN': 'Generic error code for unexpected or unclassified failures',
            'TIMEOUT': 'Error code indicating the action exceeded its maximum allowed time',
            'TF_ERROR': 'Error code indicating a transform/localization failure',
            'COLLISION_AHEAD': 'Error code indicating an obstacle was detected blocking the path',
            'INVALID_INPUT': 'Error code indicating invalid parameters were provided',
            'FAILED_TO_LOAD_BEHAVIOR_TREE': 'Error code indicating the specified behavior tree file could not be loaded',
            'FAILED_TO_DETECT_DOCK': 'Error code indicating the docking station could not be detected or located',
            'FAILED_TO_CONTROL': 'Error code indicating control system failure during docking maneuver',
            'FAILED_TO_CHARGE': 'Error code indicating charging connection or validation failed',
            'DOCK_NOT_IN_DB': 'Error code indicating the specified dock ID was not found in the dock database',
            'DOCK_NOT_VALID': 'Error code indicating the dock pose or configuration is invalid',
            'FAILED_TO_STAGE': 'Error code indicating failure to navigate to or position at the staging pose',
            
            # Feedback state constants
            'NAV_TO_STAGING_POSE': 'Status indicating navigation to the docking staging position is in progress',
            'INITIAL_PERCEPTION': 'Status indicating the robot is performing initial dock detection and perception',
            'CONTROLLING': 'Status indicating the robot is under precise control for final docking approach',
            'WAIT_FOR_CHARGE': 'Status indicating the robot is waiting for charging connection to be established',
            'CHARGING': 'Status indicating the robot is successfully connected and charging',
            'RETRY': 'Status indicating the docking process is retrying after a failed attempt',
            
            # Waypoint fields
            'missed_waypoints': 'Array of waypoints that could not be reached due to obstacles or navigation failures',
            
            # Task executor error
            'TASK_EXECUTOR_FAILED': 'Error code indicating a task executor plugin failed to execute at a waypoint',
            
            # Controller error codes
            'INVALID_CONTROLLER': 'Error code indicating the specified controller plugin is invalid or not loaded',
            'INVALID_PATH': 'Error code indicating the provided path is malformed or contains invalid data',
            'PATIENCE_EXCEEDED': 'Error code indicating the controller exceeded its patience limit waiting for progress',
            'FAILED_TO_MAKE_PROGRESS': 'Error code indicating the robot failed to make sufficient progress along the path',
            'NO_VALID_CONTROL': 'Error code indicating the controller could not compute valid control commands',
            'CONTROLLER_TIMED_OUT': 'Error code indicating the controller exceeded its maximum allowed execution time',
            
            # Planner error codes
            'INVALID_PLANNER': 'Error code indicating the specified planner plugin is invalid or not loaded',
            'START_OUTSIDE_MAP': 'Error code indicating the start position is outside the known map boundaries',
            'GOAL_OUTSIDE_MAP': 'Error code indicating the goal position is outside the known map boundaries', 
            'START_OCCUPIED': 'Error code indicating the start position is in an occupied/blocked area',
            'GOAL_OCCUPIED': 'Error code indicating the goal position is in an occupied/blocked area',
            'NO_VALID_PATH': 'Error code indicating no feasible path could be found between start and goal',
            
            # Smoother error codes
            'INVALID_SMOOTHER': 'Error code indicating the specified path smoother plugin is invalid or not loaded',
            'SMOOTHING_FAILED': 'Error code indicating the path smoothing process failed to improve the path',
            'SMOOTHER_TIMED_OUT': 'Error code indicating the smoother exceeded its maximum allowed execution time',
            'SMOOTHED_PATH_IN_COLLISION': 'Error code indicating the smoothed path would cause the robot to collide with obstacles',
            'FAILED_TO_SMOOTH_PATH': 'Error code indicating the path smoothing algorithm failed to generate a valid smoothed path',
            
            # Route planner error codes
            'INVALID_ROUTE_PLANNER': 'Error code indicating the specified route planner plugin is invalid or not loaded',
            'NO_ROUTE_FOUND': 'Error code indicating no valid route could be found between waypoints',
            'ROUTE_PLANNING_FAILED': 'Error code indicating the route planning algorithm failed',
            'NO_VALID_GRAPH': 'Error code indicating the route graph is invalid or has no connectivity',
            'INDETERMINANT_NODES_ON_GRAPH': 'Error code indicating graph nodes have ambiguous or undefined relationships',
            'NO_VALID_ROUTE': 'Error code indicating no feasible route exists between the specified waypoints',
            'INVALID_EDGE_SCORER_USE': 'Error code indicating the edge scorer plugin was used incorrectly',
            
            # GPS waypoint error codes
            'INVALID_GPS': 'Error code indicating GPS coordinates are invalid or unavailable',
            'GPS_UNRELIABLE': 'Error code indicating GPS signal quality is insufficient for navigation',
            'NO_WAYPOINTS_GIVEN': 'Error code indicating no waypoints were provided in the request',
            'STOP_ON_MISSED_WAYPOINT': 'Error code indicating the action stopped because a waypoint could not be reached',
            
            # Additional error codes
            'INVALID_GOAL': 'Error code indicating the provided goal is invalid or malformed',
            'INVALID_START': 'Error code indicating the provided start position is invalid',
            'GOAL_TOLERANCE_VIOLATED': 'Error code indicating the robot could not reach the goal within tolerance',
            'COMPUTATION_FAILED': 'Error code indicating a computational failure occurred during processing',
            
            # ID fields for waypoints and nodes
            'start_id': 'Unique identifier for the starting waypoint in the route graph',
            'goal_id': 'Unique identifier for the target waypoint in the route graph',
            'gps_poses': 'Array of GPS-based poses defining outdoor navigation waypoints with latitude/longitude coordinates',
            'goals': 'Array of target poses that the path should connect through in sequence',
            'NO_VIAPOINTS_GIVEN': 'Error code indicating no intermediate waypoints were provided for path planning through poses',
            
            # Waypoint status and tracking
            'waypoint_statuses': 'Array of status information for each waypoint including success/failure state and execution details',
            'number_of_poses_remaining': 'Count of poses/waypoints remaining to be visited in the navigation sequence',
            'last_node_id': 'Identifier of the last successfully reached node in the route graph during navigation',
            'NO_VALID_WAYPOINTS': 'Error code indicating the provided waypoints are invalid or unreachable',
            
            # Operation status
            'OPERATION_FAILED': 'Error code indicating a general operation failure occurred during execution',
            'execution_duration': 'Total time taken for the route computation and tracking operation to complete',
            'next_node_id': 'Identifier of the next node to be visited in the route graph during navigation',
            'current_edge_id': 'Identifier of the current edge being traversed in the route graph',
            'operations_triggered': 'List of navigation operations or behaviors that have been triggered during route execution',
            'rerouted': 'Flag indicating whether the route has been dynamically recalculated due to obstacles or changes',
            
            # Path smoothing fields
            'smoother_id': 'Name of the path smoothing algorithm plugin to use',
            'max_smoothing_duration': 'Maximum time allowed for the path smoothing algorithm to compute and refine the path',
            'check_for_collisions': 'Whether to perform collision checking on the smoothed path to ensure safety',
            'smoothing_duration': 'Actual time taken by the smoothing algorithm to process and optimize the path in seconds',
            'was_completed': 'Whether the smoothing operation finished successfully within the allocated time and computational limits',
            
            # Path following fields  
            'progress_checker_id': 'Name of the progress monitoring plugin to use for tracking path following advancement',
            'distance_to_goal': 'Current distance from the robot to the final goal position in meters, updated continuously during navigation',
            
            # Spin behavior fields
            'angular_distance_traveled': 'Total angular distance the robot has rotated during the spin action in radians (cumulative measurement)',
            
            # Wait behavior fields  
            'time_left': 'Remaining wait time before the wait action completes, in seconds (counts down to zero)',
            
            # Docking fields
            'dock_type': 'Type of docking station or charging platform to dock with (e.g., "nova_carter_dock", "charging_dock")',
            'use_dock_id': 'Whether to use the dock_id field to identify the dock from a database, or use the dock_pose field for direct positioning',
            
            # Other common fields  
            'success': 'Boolean flag indicating whether the operation completed successfully',
            'num_retries': 'Count of retry attempts made during the action execution',
            'current_waypoint': 'Index of the current waypoint being pursued in a sequence'
        }
    
    def _get_field_description(self, field: ActionField) -> str:
        """Get description for a field, using mapping if no comment exists."""
        if field.comment and field.comment.strip():
            comment = field.comment.strip()
            # Handle specific prefixed patterns by removing the prefix first
            if comment.startswith('goal definition. '):
                cleaned_comment = comment.replace('goal definition. ', '')
                if cleaned_comment.strip():
                    return cleaned_comment
            elif comment.startswith('result definition. '):
                cleaned_comment = comment.replace('result definition. ', '')
                if cleaned_comment.strip():
                    return cleaned_comment
            elif comment.startswith('feedback definition. '):
                cleaned_comment = comment.replace('feedback definition. ', '')  
                if cleaned_comment.strip():
                    return cleaned_comment
            
            # Skip generic section headers and other prefixed comments
            if comment not in ['goal definition', 'result definition', 'feedback definition'] and \
               not comment.startswith('goal definition') and \
               not comment.startswith('result definition') and \
               not comment.startswith('feedback definition'):
                return comment
        
        # Try to find a description based on field name
        if field.field_name in self.field_descriptions:
            return self.field_descriptions[field.field_name]
        
        # Try based on field type patterns
        if 'PoseStamped' in field.type_name:
            return 'Pose with header information (frame_id and timestamp)'
        elif 'Duration' in field.type_name:
            return 'Duration parameter - specifies time limits for action completion, timeout controls, or reports elapsed/remaining time for performance monitoring and safety'
        elif 'Empty' in field.type_name:
            return 'Empty message (no data fields)'
        elif field.type_name in ['string', 'std_msgs/String']:
            return 'String identifier - used to specify plugin names (planner_id, controller_id), behavior tree files, dock identifiers, or configuration keys for runtime algorithm selection'
        elif field.type_name in ['float32', 'float64']:
            return 'Float parameter - represents physical measurements like distances (meters), velocities (m/s), angles (radians), or progress values for precise motion control and feedback'
        elif field.type_name in ['int16', 'int32', 'int64', 'uint16', 'uint32', 'uint64']:
            return 'Integer parameter - represents counts, indices, or identifiers for navigation elements like waypoint numbers, node IDs, or status codes'
        elif field.type_name == 'bool':
            return 'Boolean toggle - controls optional behaviors like collision checking, coordinate frame usage (current vs provided pose), or feature enablement (staging poses, dock ID usage)'
        
        return "Parameter for the action - enables runtime configuration of navigation behavior, plugin selection, safety controls, progress monitoring, and performance tuning (see specific action documentation for parameter details)"

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