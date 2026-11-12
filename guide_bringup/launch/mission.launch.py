"""Full mission in simulation: one action goal takes the robot to a destination.

Starts: sim robot, localizer, route follower (no route; routes come from the
mission manager), goal sender, route planner (cached OSM graph) and mission
manager (curated destinations).

Usage:
  ros2 launch guide_bringup mission.launch.py
  ros2 launch guide_bringup mission.launch.py rviz:=true
  ros2 launch guide_bringup mission.launch.py start_x:=... start_y:=... start_yaw_deg:=...

Then, in another terminal:
  ros2 action send_goal --feedback /guide/navigate_to guide_interfaces/action/NavigateTo \
      "{destination: 'south ken'}"
  ros2 service call /guide/user_go std_srvs/srv/Trigger      # the user's go
  ros2 service call /guide/user_pause std_srvs/srv/Trigger   # the user's pause

Ctrl+C stops all nodes (the goal sender cancels any active goal first).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ros2 launch calls this function and starts everything in the returned list.

    # Files installed with this package (share/guide_bringup/...). FindPackageShare
    # finds that folder at launch time.
    share = FindPackageShare('guide_bringup')

    # Launch arguments: values the user can override on the command line (name:=value).
    args = [
        # The robot must start on a walkway (the planner snaps the start to one);
        # the default is the start of the demo route to South Kensington Station.
        DeclareLaunchArgument('start_x', default_value='-26.86',
                              description='Sim robot start, metres east of the origin'),
        DeclareLaunchArgument('start_y', default_value='-19.04',
                              description='Sim robot start, metres north of the origin'),
        DeclareLaunchArgument('start_yaw_deg', default_value='-82.0',
                              description='Sim robot start heading, degrees (0 = east)'),
        # The ENU origin of the map frame. Given ONCE here and passed to every node
        # that converts between lat/lon and metres, so they cannot disagree.
        DeclareLaunchArgument('origin_lat', default_value='51.4988',
                              description='Latitude of the map frame origin (degrees)'),
        DeclareLaunchArgument('origin_lon', default_value='-0.1749',
                              description='Longitude of the map frame origin (degrees)'),
        DeclareLaunchArgument(
            'graph_file',
            default_value=PathJoinSubstitution([share, 'osm', 'south_ken_walk.graphml']),
            description='Cached OSM walking graph (.graphml from fetch_osm_graph)'),
        DeclareLaunchArgument(
            'destinations_file',
            default_value=PathJoinSubstitution([share, 'destinations', 'campus.yaml']),
            description='Curated destination list for the mission manager'),
        DeclareLaunchArgument('rviz', default_value='false',
                              description="'true' also starts RViz"),
    ]

    # Launch arguments are text. ParameterValue(..., value_type=float) makes sure
    # '-82' or '51.4988' arrives as a float, because the nodes declare these
    # parameters as floats and ROS rejects an int (or a string) for a float.
    def as_float(name):
        return ParameterValue(LaunchConfiguration(name), value_type=float)

    origin = {'origin_lat': as_float('origin_lat'), 'origin_lon': as_float('origin_lon')}

    # Each Node action starts one executable (the same as 'ros2 run package executable').
    # output='screen' shows its log lines in this terminal, prefixed with the
    # process name, e.g. [mission_manager-6]. parameters=[{...}] is the launch-file
    # equivalent of --ros-args -p name:=value.
    sim = Node(
        package='guide_sim', executable='sim_robot', output='screen',
        parameters=[{
            'start_x': as_float('start_x'),
            'start_y': as_float('start_y'),
            'start_yaw_deg': as_float('start_yaw_deg'),
            **origin,
        }])
    localizer = Node(package='guide_nodes', executable='localizer', output='screen',
                     parameters=[origin])
    # No route_file: the follower waits for routes on guide/set_route.
    route_follower = Node(package='guide_nodes', executable='route_follower',
                          output='screen')
    goal_sender = Node(package='guide_nodes', executable='goal_sender', output='screen')
    route_planner = Node(
        package='guide_nodes', executable='route_planner', output='screen',
        parameters=[{'graph_file': LaunchConfiguration('graph_file'), **origin}])
    mission_manager = Node(
        package='guide_nodes', executable='mission_manager', output='screen',
        parameters=[{'destinations_file': LaunchConfiguration('destinations_file'),
                     **origin}])

    # Optional RViz with the planner's configuration (OSM graph, planned route).
    # condition=IfCondition(...) starts the node only if the argument is 'true'.
    rviz = Node(
        package='rviz2', executable='rviz2', output='screen',
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'route_planner.rviz'])],
        condition=IfCondition(LaunchConfiguration('rviz')))

    return LaunchDescription([
        *args,
        sim,
        localizer,
        route_follower,
        goal_sender,
        route_planner,
        mission_manager,
        rviz,
    ])