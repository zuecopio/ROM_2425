
# cartographer.py

"""
This launch file integrates the CoppeliaSim simulator with the Cartographer
SLAM system for the TurtleBot3 robot. It allows for configurable SLAM
resolution and is designed to work within the 'little_warehouse' package.

Main functionalities:
- Declares a configurable launch argument for the Cartographer SLAM resolution.
- Includes the CoppeliaSim simulation launch file (without RViz2).
- Includes the Cartographer SLAM launch file with the specified resolution.
- Groups all actions into a single launch description for streamlined execution.

Prerequisites:
- The 'little_warehouse' package must contain the required simulation launch
  files.
- The 'turtlebot3_cartographer' package must be installed and accessible.
- The environment variable TURTLEBOT3_MODEL must be set appropriately.

Authors:
- Marcos Belda Martinez <mbelmar@etsinf.upv.es>
- Angela Espert Cornejo <aespcor@etsinf.upv.es>
- Lourdes Frances Limmera <lfralli@epsa.upv.es>

Date: 2025-06-17

Based on:
- Standard ROS 2 launch mechanisms and integration patterns for SLAM and
  simulation environments.
"""

# ---------------------------------------------------------------------------- #
# NEEDED LIBRARIES

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import GroupAction
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# ---------------------------------------------------------------------------- #
# GENERATE LAUNCH FUNCTION

def generate_launch_description() -> LaunchDescription:
    """
    Generates the launch description for integrating CoppeliaSim and 
    Cartographer SLAM with a configurable resolution parameter.

    :rtype: LaunchDescription
    :return: A LaunchDescription object that launches both the CoppeliaSim
             simulation and the Cartographer SLAM system with a configurable
             resolution parameter, grouped into a single launch action.
    """

    # Get the path to the launch directory of
    # the 'little_warehouse' package
    coppeliasim_pkg = get_package_share_directory('little_warehouse')
    coppeliasim_dir = os.path.join(coppeliasim_pkg, 'launch')

    # Get the path to the launch directory of
    # the 'turtlebot3_cartographer' package
    cartographer_pkg = get_package_share_directory('turtlebot3_cartographer')
    cartographer_dir = os.path.join(cartographer_pkg, 'launch')

    # Define the 'resolution' launch argument
    resolution = LaunchConfiguration('resolution')

    declare_resolution_cmd = DeclareLaunchArgument(
        'resolution',
        default_value='0.001',
        description='Resolution for the Cartographer SLAM algorithm'
    )

    # Group all launch actions together
    cartographer_cmd = GroupAction([

        # Declare the resolution parameter
        declare_resolution_cmd,

        # Include the CoppeliaSim launch file without RViz2
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    coppeliasim_dir, 'coppeliasim_no_rviz2.launch.py'
                )
            ),
        ),

        # Include the Cartographer launch file with the resolution argument
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    cartographer_dir, 'cartographer.launch.py'
                )
            ),
            launch_arguments={'resolution': resolution}.items()
        ),
    ])

    # Create the full launch description
    ld = LaunchDescription()

    # Add the grouped actions (CoppeliaSim + Cartographer)
    ld.add_action(cartographer_cmd)

    return ld

### end of file ###
