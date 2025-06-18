# navigation_with_speed_limit.launch.py

"""
This launch file initializes the navigation stack using a specified map and
parameter file. It also sets an initial pose to assist with localization.

Main functionalities:
- Declares configurable launch arguments for simulation time, map, and
  parameters.
- Publishes an initial pose to the /initialpose topic to initialize
  localization.
- Includes the standard Navigation2 bringup launch file.
- Launches RViz with a custom configuration for visualization.

Prerequisites:
- The 'little_warehouse' package must be installed and contain the required
  maps, parameters, and RViz configuration files.
- The 'nav2_bringup' package must be available to launch the navigation stack.

Authors:
- Marcos Belda Martinez <mbelmar@etsinf.upv.es>
- Angela Espert Cornejo <aespcor@etsinf.upv.es>
- Lourdes Frances Limmera <lfralli@epsa.upv.es>

Date: 2025-06-17

Based on:
- navigation2.launch.py (TurtleBot3 Navigation2 package)
  Author: Darby Lim
  Copyright 2019 Open Source Robotics Foundation, Inc.
  Licensed under the Apache License, Version 2.0

This file has been adapted and extended to meet the specific requirements of the
'little_warehouse' package, including custom map, parameters, initial pose
publication, and RViz configuration.
"""

# ---------------------------------------------------------------------------- #
# NEEDED LIBRARIES

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------------------------------------------------------------------- #
# GENERATE LAUNCH FUNCTION

def generate_launch_description() -> LaunchDescription:
    """
    Generates the launch description for starting the navigation stack
    with a specified map, parameter file, and initial pose for localization.

    :rtype: LaunchDescription
    :return: A complete launch description for the navigation stack

    """
    # Get the path to the 'little_warehouse' package
    little_warehouse_pkg = get_package_share_directory('little_warehouse')

    # STEP 1 -------------------------------------------------------------------
    # Declare launch configurations for simulation time, map, and
    # parameters file
    
    use_sim_time = LaunchConfiguration(
        'use_sim_time',
        default='false'
    )

    map_dir = LaunchConfiguration(
        'map',
        default=os.path.join(
            little_warehouse_pkg,
            'maps',
            'coppeliasim_map.yaml'
        )
    )
    
    params_file_dir = LaunchConfiguration(
        'params_file',
        default=os.path.join(
            little_warehouse_pkg,
            'params',
            'nav2_params_speed_limit.yaml'
        )
    )

    # STEP 2 -------------------------------------------------------------------
    # Define paths to external launch files and RViz configuration
    
    nav2_launch_file_dir = os.path.join(
        get_package_share_directory('nav2_bringup'),
        'launch'
    )

    rviz_config_dir = os.path.join(
        little_warehouse_pkg,
        'rviz',
        'navigation_with_speed_limit.rviz'
    )

    # STEP 3 -------------------------------------------------------------------
    # Declare launch arguments to expose them to the command line

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=params_file_dir,
        description='Full path to param file to load'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    # STEP 4 -------------------------------------------------------------------
    # Group actions to publish initial pose and launch navigation stack

    localization_cmd = GroupAction([

        # Publish initial pose for localization
        ExecuteProcess(
            cmd=[[
                'ros2 topic pub --once /initialpose geometry_msgs/msg/'
                'PoseWithCovarianceStamped ',
                '"""{header: {frame_id: \'map\'}, pose: {pose: {position: {x: '
                '0.0, y: 0.0, z: 0.0}, ',
                'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, ',
                'covariance: [0.01,0.0,0.0,0.0,0.0,0.0, ',
                '0.0,0.01,0.0,0.0,0.0,0.0, ',
                '0.0,0.0,0.0,0.0,0.0,0.0, ',
                '0.0,0.0,0.0,0.0,0.0,0.0, ',
                '0.0,0.0,0.0,0.0,0.0,0.0, ',
                '0.0,0.0,0.0,0.0,0.0,0.0076]} }"""'
            ]],
            shell=True
        ),

        # Declare input arguments
        declare_map_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,

        # Include the main Navigation2 bringup launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [nav2_launch_file_dir, '/bringup_launch.py']
            ),
            launch_arguments={
                'map': map_dir,
                'use_sim_time': use_sim_time,
                'params_file': params_file_dir
            }.items(),
        ),

        # Launch RViz with the specified configuration
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_dir],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
    ])

    # Create and return the complete launch description
    ld = LaunchDescription()
    ld.add_action(localization_cmd)

    return ld

### end of file ###