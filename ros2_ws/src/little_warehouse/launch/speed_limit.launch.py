# speed_limit.launch.py

"""
This launch file sets up the speed limit functionality in the Nav2 navigation
stack. It configures the necessary nodes and parameters to manage speed limits
for the robot's navigation.

Main functionalities:
- Loads the speed limit parameters from a YAML file.
- Configures the speed limit server node.
- Handles lifecycle transitions for speed management.
- Supports both regular and composable node execution.

Prerequisites:
- The 'nav2_map_server' package must be installed and properly configured.
- The parameter YAML file for speed limits must be provided.

Authors:
- Marcos Belda Martinez <mbelmar@etsinf.upv.es>
- Angela Espert Cornejo <aespcor@etsinf.upv.es>
- Lourdes Frances Limmera <lfralli@epsa.upv.es>

Date: 2025-06-17

Based on:
- Code from the repository: https://github.com/larmesto/RM_prac
  Authors:
   - Leopoldo Armesto
   - Ricardo Nuñez

This file has been adapted and extended to meet the specific requirements
of the speed limit functionality in the Nav2 navigation stack.
"""

# ---------------------------------------------------------------------------- #
# NEEDED LIBRARIES

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LoadComposableNodes
from launch_ros.actions import PushRosNamespace
from launch_ros.descriptions import ComposableNode
from nav2_common.launch import RewrittenYaml

# ---------------------------------------------------------------------------- #
# GENERATE LAUNCH FUNCTION

def generate_launch_description() -> LaunchDescription:
    """
    Generates a launch description to bring up the costmap filter functionality
    in the Nav2 navigation stack.

    This launch file:
    - Loads the filter mask map.
    - Starts map and info servers for costmap filters.
    - Handles lifecycle transitions.
    - Supports both regular and composable node execution.

    :rtype: LaunchDescription
    :return: A LaunchDescription object that contains the configuration for
             launching the costmap filter nodes and lifecycle manager.

    """
    # Lifecycle-managed node names
    lifecycle_nodes = ['filter_mask_server', 'costmap_filter_info_server']

    # Launch arguments and configurations
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    mask_yaml_file = LaunchConfiguration('mask')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    container_name_full = (namespace, '/', container_name)

    # Declare all launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically startup the nav2 stack'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        description='Full path to the ROS2 parameters file to use'
    )

    declare_mask_yaml_file_cmd = DeclareLaunchArgument(
        'mask',
        description='Full path to filter mask yaml file to load'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition', default_value='True',
        description='Use composed bringup if True'
    )

    declare_container_name_cmd = DeclareLaunchArgument(
        'container_name', default_value='nav2_container',
        description='The name of container that nodes will load in if use '
                    'composition'
    )

    # Prepare rewritten parameter YAML file
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'yaml_filename': mask_yaml_file
    }

    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True
    )

    # GroupAction: launch regular (non-composable) nodes
    load_nodes = GroupAction(
        condition=IfCondition(
            PythonExpression(
                ['not ', use_composition]
            )
        ),
        actions=[
            Node(
                package='nav2_map_server',
                executable='map_server',
                name='filter_mask_server',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[configured_params]
            ),
            Node(
                package='nav2_map_server',
                executable='costmap_filter_info_server',
                name='costmap_filter_info_server',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[configured_params]
            ),
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_costmap_filters',
                namespace=namespace,
                output='screen',
                emulate_tty=True,
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes}
                ]
            )
        ]
    )

    # GroupAction: load composable nodes into a container
    load_composable_nodes = GroupAction(
        condition=IfCondition(
            use_composition
        ),
        actions=[
            PushRosNamespace(
                condition=IfCondition(
                    PythonExpression(
                        ["'", LaunchConfiguration('namespace'), "' != ''",]
                    )
                ),
                namespace=namespace
            ),
            LoadComposableNodes(
                target_container=container_name_full,
                composable_node_descriptions=[
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::MapServer',
                        name='filter_mask_server',
                        parameters=[configured_params]
                    ),
                    ComposableNode(
                        package='nav2_map_server',
                        plugin='nav2_map_server::CostmapFilterInfoServer',
                        name='costmap_filter_info_server',
                        parameters=[configured_params]
                    ),
                    ComposableNode(
                        package='nav2_lifecycle_manager',
                        plugin='nav2_lifecycle_manager::LifecycleManager',
                        name='lifecycle_manager_costmap_filters',
                        parameters=[
                            {'use_sim_time': use_sim_time},
                            {'autostart': autostart},
                            {'node_names': lifecycle_nodes}
                        ]
                    ),
                ]
            )
        ]
    )

    # Final launch description
    ld = LaunchDescription()

    # Declare all arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_mask_yaml_file_cmd)
    ld.add_action(declare_use_composition_cmd)
    ld.add_action(declare_container_name_cmd)

    # Add conditionally-composed or regular node loading
    ld.add_action(load_nodes)
    ld.add_action(load_composable_nodes)

    return ld

### end of file ###