"""Simulation run: sim robot + localizer + route follower + goal sender.

Usage:
  ros2 launch guide_bringup sim_follow.launch.py
  ros2 launch guide_bringup sim_follow.launch.py route_file:=/path/to/route.yaml

The robot follows the route on its own until the route follower logs "Arrived".
Ctrl+C stops all nodes (the goal sender cancels any active goal first).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ros2 launch calls this function and starts everything in the returned list.

    # Launch argument: a value the user can override on the command line
    # (route_file:=...). The default is the L route installed with this package.
    # FindPackageShare finds install/guide_bringup/share/guide_bringup at launch
    # time, the same folder 'ros2 pkg prefix' pointed to in the manual runs.
    route_file_arg = DeclareLaunchArgument(
        'route_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('guide_bringup'), 'routes', 'l_route.yaml']),
        description='Route YAML file (frame_id: map, waypoints in metres)')

    # Each Node action starts one executable (the same as 'ros2 run package executable').
    # output='screen' shows its log lines in this terminal, prefixed with the
    # process name, e.g. [route_follower-3].
    sim = Node(package='guide_sim', executable='sim_robot', output='screen')
    localizer = Node(package='guide_nodes', executable='localizer', output='screen')

    # parameters=[{...}] is the launch-file equivalent of --ros-args -p name:=value.
    # LaunchConfiguration('route_file') is replaced by the argument's value at launch.
    route_follower = Node(
        package='guide_nodes', executable='route_follower', output='screen',
        parameters=[{'route_file': LaunchConfiguration('route_file')}])

    goal_sender = Node(package='guide_nodes', executable='goal_sender', output='screen')

    return LaunchDescription([
        route_file_arg,
        sim,
        localizer,
        route_follower,
        goal_sender,
    ])