"""Starts the navigation nodes: places_node, people_node and feedback_node.

Nav2 and localisation must already be running; navigation_sim.launch.py starts everything in Gazebo."""

# this launch file just starts our 3 python nodes, and passes them the launch arguments as ROS parameters.
# you can give the arguments on the command line, e.g. ros2 launch .../navigation.launch.py use_sim_time:=true

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def node(script, *params):
    """Run one of the node scripts, giving it the launch arguments named in params as ROS parameters."""
    parameter_types = {'use_sim_time': bool, 'tracker_domain': int}
    parameters = {name: ParameterValue(LaunchConfiguration(name), value_type=parameter_types.get(name, str)) for name in params}
    return Node(package='navigation_nodes', executable=script.removesuffix('.py'), parameters=[parameters], output='screen')


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
        DeclareLaunchArgument('tracker_domain', default_value='-1', description='ROS domain of the people tracker, -1 = ours'),
        node('places_node.py', 'places_file', 'use_sim_time', 'map_frame', 'base_frame'),
        node('people_node.py', 'use_sim_time', 'map_frame', 'camera_frame', 'fake_people', 'tracker_domain'),
        node('feedback_node.py', 'use_sim_time', 'map_frame', 'base_frame'),
    ])
