#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """
    Launch file for the X500 Planning Service
    """
    
    # Declare launch arguments
    service_name_arg = DeclareLaunchArgument(
        'service_name',
        default_value='x500_planner',
        description='Service name for trajectory planning'
    )
    
    planning_group_arg = DeclareLaunchArgument(
        'planning_group',
        default_value='x500_group',
        description='MoveIt planning group name'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='INFO',
        description='Log level (DEBUG, INFO, WARN, ERROR)'
    )
    
    # Get launch configurations
    service_name = LaunchConfiguration('service_name')
    planning_group = LaunchConfiguration('planning_group')
    log_level = LaunchConfiguration('log_level')
    
    # X500 Planning Service Node
    planning_service_node = Node(
        package='x500_trajectory_planner',
        executable='x500_planning_service_node',
        name='x500_planning_service_node',
        output='screen',
        parameters=[{
            'service_name': service_name,
            'planning_group': planning_group,
        }],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True,
    )
    
    # Log startup message
    startup_msg = LogInfo(
        msg=[
            '🚁 Starting X500 Planning Service...\n',
            '   Service name: ', service_name, '\n',
            '   Planning group: ', planning_group, '\n',
            '   Log level: ', log_level
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        service_name_arg,
        planning_group_arg,
        log_level_arg,
        
        # Startup message
        startup_msg,
        
        # Nodes
        planning_service_node,
    ])


if __name__ == '__main__':
    generate_launch_description()