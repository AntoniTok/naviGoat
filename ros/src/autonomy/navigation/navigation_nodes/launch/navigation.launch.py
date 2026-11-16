"""Starts the navigation nodes: places_node, people_node and feedback_node.

Nav2 and localisation must already be running; navigation_sim.launch.py starts everything in Gazebo."""

# this launch file just starts our 3 python nodes, and passes them the launch arguments as ROS parameters.
# you can give the arguments on the command line, e.g. ros2 launch .../navigation.launch.py use_sim_time:=true

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration

# the navigation_nodes folder (this file is in navigation_nodes/launch/, so it's two dirnames up)
NODES_DIR = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))


def node(script, *params):
    """Run one of the node scripts, giving it the launch arguments named in params as ROS parameters."""
    # this builds the same command as typing:  python3 places_node.py --ros-args -p places_file:=... -p ...
    cmd = ['python3', os.path.join(NODES_DIR, script), '--ros-args']
    for name in params:
        cmd += ['-p', [f'{name}:=', LaunchConfiguration(name)]]  # LaunchConfiguration(name) = that argument's value
    return ExecuteProcess(cmd=cmd, output='screen')  # output='screen' shows the node's prints in this terminal


def generate_launch_description():
    # ros2 launch calls this function. first the arguments you can pass (name:=value) with their defaults,
    # then the three nodes and which arguments each one gets
    return LaunchDescription([
        DeclareLaunchArgument('places_file', default_value=os.path.expanduser('~/.ros/navigoat_places.yaml')),
        DeclareLaunchArgument('use_sim_time', default_value='false'),  # true in Gazebo, so the nodes use its clock
        DeclareLaunchArgument('map_frame', default_value='map'),
        DeclareLaunchArgument('base_frame', default_value='base_link'),
        DeclareLaunchArgument('camera_frame', default_value='camera_link'),
        DeclareLaunchArgument('fake_people', default_value='none', description='YAML file of made-up people, for testing'),
        node('places_node.py', 'places_file', 'use_sim_time', 'map_frame', 'base_frame'),
        node('people_node.py', 'use_sim_time', 'map_frame', 'camera_frame', 'fake_people'),
        node('feedback_node.py', 'use_sim_time', 'map_frame', 'base_frame'),
    ])
