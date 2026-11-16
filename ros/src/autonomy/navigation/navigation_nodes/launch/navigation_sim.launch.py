"""The whole navigation pipeline in Gazebo: Gazebo, RTAB-Map, Nav2 and RViz (from simulation.launch.py), plus the
navigation nodes.

Map first (mode:=mapping), then navigate on the saved map:

    ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_sim.launch.py mode:=mapping
    ros2 launch /workspace/ros/src/autonomy/navigation/navigation_nodes/launch/navigation_sim.launch.py

With made-up people to test the social layer (edit the file to put them where your map has room):

    ros2 launch .../navigation_sim.launch.py fake_people:=/workspace/ros/src/autonomy/navigation/config/fake_people.yaml

The places are kept in ros/src/simulation/maps/<world>_places.yaml, next to <world>.db, the map they belong to.
"""

# the full simulation in one command: the team's simulation.launch.py (Gazebo + RTAB-Map + Nav2 + RViz),
# plus our navigation nodes from navigation.launch.py

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

LAUNCH_DIR = os.path.dirname(os.path.realpath(__file__))  # navigation_nodes/launch
SRC_DIR = os.path.abspath(os.path.join(LAUNCH_DIR, '..', '..', '..', '..'))  # ros/src


def launch_navigation(context):
    # we need the world's name as normal text to make the places file name ("indoor.sdf" -> "indoor_places.yaml").
    # launch arguments only turn into text inside an OpaqueFunction, that's why this is a separate function
    world = os.path.splitext(os.path.basename(LaunchConfiguration('world').perform(context)))[0]
    # start our nodes (navigation.launch.py) with these arguments
    return [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(LAUNCH_DIR, 'navigation.launch.py')),
        launch_arguments={
            'places_file': os.path.join(SRC_DIR, 'simulation', 'maps', f'{world}_places.yaml'),
            'use_sim_time': 'true',  # in Gazebo everything runs on the simulation's clock
            'fake_people': LaunchConfiguration('fake_people'),
        }.items(),
    )]


def generate_launch_description():
    return LaunchDescription([
        # Localisation by default: Nav2 needs a finished map, and saved places only make sense on a map that stays the same.
        DeclareLaunchArgument('mode', default_value='localisation', choices=['mapping', 'localisation']),
        DeclareLaunchArgument('world', default_value='indoor.sdf'),
        DeclareLaunchArgument('fake_people', default_value='none', description='YAML file of made-up people, for testing'),
        # Gazebo, RTAB-Map, Nav2 and RViz, from the simulation folder
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(SRC_DIR, 'simulation', 'launch', 'simulation.launch.py')),
            launch_arguments={'mode': LaunchConfiguration('mode'), 'world': LaunchConfiguration('world')}.items(),
        ),
        # and our nodes
        OpaqueFunction(function=launch_navigation),
    ])
