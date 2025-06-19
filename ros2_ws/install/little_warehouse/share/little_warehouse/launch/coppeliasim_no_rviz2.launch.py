
# coppeliasim_no_rviz2.launch.py

"""
This launch file initializes a simulation of the TurtleBot3 robot in
CoppeliaSim without launching RViz2. It is designed for use within the
'little_warehouse' package and supports flexible configuration through
launch arguments.

Main functionalities:
- Declares configurable launch arguments for the simulation scene and
  parameter files.
- Launches the CoppeliaSim simulator with a predefined scene file.
- Starts the `robot_state_publisher` node to publish the robot's TF and URDF
  based on the selected TurtleBot3 model.

Prerequisites:
- The 'little_warehouse' package must include the required URDF, parameter,
  and scene files.
- The environment variable TURTLEBOT3_MODEL must be set to a valid model
  (e.g., 'burger', 'waffle', or 'waffle_pi').
- CoppeliaSim must be installed and accessible via the `coppeliaSim.sh` script.

Authors:
- Marcos Belda Martinez <mbelmar@etsinf.upv.es>
- Angela Espert Cornejo <aespcor@etsinf.upv.es>
- Lourdes Frances Limmera <lfralli@epsa.upv.es>

Date: 2025-06-17

Based on:
- Standard ROS 2 launch practices and TurtleBot3 simulation setup.
  Adapted to integrate with CoppeliaSim and exclude RViz2 for lightweight
  simulation scenarios.
"""

# ---------------------------------------------------------------------------- #
# NEEDED LIBRARIES

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.actions import ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ---------------------------------------------------------------------------- #
# GLOBAL PARAMETERS

# Get the TurtleBot3 model from the environment variable
TURTLEBOT3_MODEL = os.environ['TURTLEBOT3_MODEL']

# Define the ROS 2 package name
PKG_NAME = 'little_warehouse'

# ---------------------------------------------------------------------------- #
# GENERATE LAUNCH FUNCTION

def generate_launch_description() -> LaunchDescription:
    """
    Generates a launch description that initializes a simulation environment 
    for TurtleBot3 in CoppeliaSim, without launching RViz2.

    This launch file:
    - Declares launch arguments for selecting the parameter file and scene file.
    - Launches CoppeliaSim with the specified scene.
    - Starts the robot_state_publisher node with the appropriate URDF file.
    - Can optionally launch other simulation nodes (currently commented).

    :rtype: LaunchDescription
    :return: A LaunchDescription object that sets up the simulation environment
             in CoppeliaSim for a TurtleBot3 robot. It includes the necessary
             launch arguments, starts the simulator with the specified scene,
             and launches the robot_state_publisher node to broadcast the
             robot's state using the appropriate URDF model.
    """

    # Launch configuration for the parameter file path
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory(PKG_NAME),
            'param',
            TURTLEBOT3_MODEL + '.yaml'
        )
    )

    # Launch configuration for the CoppeliaSim scene file path
    scene_dir = LaunchConfiguration(
        'scene_dir',
        default=os.path.join(
            get_package_share_directory(PKG_NAME),
            'scenes',
            'coppeliasim_scene.ttt'
        )
    )

    # Configuration to enable simulation time
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # Path to the URDF file of the selected TurtleBot3 model
    urdf_file_name = f'turtlebot3_{TURTLEBOT3_MODEL}.urdf'
    urdf = os.path.join(
        get_package_share_directory(PKG_NAME),
        'urdf',
        urdf_file_name
    )

    return LaunchDescription([
        # Log info when the simulation starts
        LogInfo(msg=['Execute Turtlebot3 CoppeliaSim!!']),

        # Declare the launch argument for the parameter file
        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Specifying parameter direction'
        ),

        # Declare the launch argument for the CoppeliaSim scene file
        DeclareLaunchArgument(
            'scene_dir',
            default_value=scene_dir,
            description='Specifying scenes path'
        ),

        # Start the CoppeliaSim simulation with the selected scene        
        ExecuteProcess(
            cmd=[['coppeliaSim.sh -f ', scene_dir, ' -s 0']],
            shell=True
        ),

        # Launch robot_state_publisher to publish the robot's transforms        
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            arguments=[urdf]
        ),
    ])

### end of file ###