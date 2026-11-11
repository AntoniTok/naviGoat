"""Command-line client for the guide/plan_route service.

Plans a route, prints a summary and can save it as a route YAML that the
route follower loads (route_file parameter).

Usage:
  ros2 run guide_nodes plan_route START_LAT START_LON GOAL_LAT GOAL_LON [--save FILE]
"""

import argparse
import os
import sys
import math

import rclpy
import yaml
from rclpy.node import Node
from rclpy.utilities import remove_ros_args

from guide_interfaces.msg import RouteCrossing
from guide_interfaces.srv import PlanRoute

# RouteCrossing.msg's uint8 constants -> readable names.
TYPE_NAMES = {
    RouteCrossing.SIGNALLED: 'signalled',
    RouteCrossing.UNSIGNALLED: 'unsignalled',
    RouteCrossing.UNKNOWN: 'unknown',
}


def parse_args(argv):
    """Read the command-line options (ROS arguments already removed)."""
    parser = argparse.ArgumentParser(description='Plan a walking route via guide/plan_route.')
    parser.add_argument('start_lat', type=float)
    parser.add_argument('start_lon', type=float)
    parser.add_argument('goal_lat', type=float)
    parser.add_argument('goal_lon', type=float)
    parser.add_argument('--save', metavar='FILE',
                        help='save the route as a route YAML for the route follower')
    parser.add_argument('--timeout', type=float, default=10.0,
                        help='seconds to wait for the service and for the answer (default 10)')
    return parser.parse_args(argv)


def save_route_yaml(path, response):
    """Write the route in the follower's format: {frame_id, waypoints: [[x, y], ...]}.

    Crossings are written as comments for now (the route format gains a
    crossings key in Step 8b).
    """
    waypoints = [[round(p.pose.position.x, 3), round(p.pose.position.y, 3)]
                 for p in response.path.poses]
    header = ['# Planned by route_planner (guide/plan_route).',
              f'# {response.message}',
              '# Crossings (distance along route, m), informational until Step 8b:']
    header += [f'#   {c.start_s:.1f} .. {c.end_s:.1f}: {TYPE_NAMES[c.crossing_type]}, '
               f'{c.crossed_road} road' for c in response.crossings] or ['#   none']
    body = yaml.safe_dump({'frame_id': response.path.header.frame_id, 'waypoints': waypoints},
                          default_flow_style=None, sort_keys=False)
    with open(path, 'w', encoding='utf-8') as f:
        f.write('\n'.join(header) + '\n' + body)
    return len(waypoints)


def main(argv=None):
    """Call the service once; return 0 on success, 1 on any failure."""
    argv = sys.argv if argv is None else argv
    # remove_ros_args drops "--ros-args ..." so argparse only sees our own arguments.
    args = parse_args(remove_ros_args(argv)[1:])

    rclpy.init(args=argv)
    node = Node('plan_route_cli')
    # A service client: create_client(type, name) must match the server's type
    # and name exactly, or the call never connects.
    client = node.create_client(PlanRoute, 'guide/plan_route')
    try:
        if not client.wait_for_service(timeout_sec=args.timeout):
            print(f'Service guide/plan_route not available after {args.timeout:.0f} s. '
                  'Is route_planner running (and this terminal sourced)?', file=sys.stderr)
            return 1

        request = PlanRoute.Request(start_latitude=args.start_lat, start_longitude=args.start_lon,
                                    goal_latitude=args.goal_lat, goal_longitude=args.goal_lon)
        # call_async sends the request and returns a "future" that is filled
        # in when the answer arrives; spinning the node lets that happen.
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=args.timeout)
        if not future.done():
            print(f'No answer within {args.timeout:.0f} s.', file=sys.stderr)
            return 1
        response = future.result()

        if not response.success:
            print(f'FAILED: {response.message}')
            return 1
        print(response.message)
        print(f'Snapped: start {response.start_snap_distance:.1f} m, '
              f'goal {response.goal_snap_distance:.1f} m from the requested points')
        for c in response.crossings:
            print(f'  crossing at s = {c.start_s:.1f} .. {c.end_s:.1f} m: '
                  f'{TYPE_NAMES[c.crossing_type]}, {c.crossed_road} road, '
                  f'tactile paving: {c.tactile_paving}')
        first, last = response.path.poses[0].pose.position, response.path.poses[-1].pose.position
        print(f'Map frame: starts at ({first.x:.1f}, {first.y:.1f}) m, '
              f'ends at ({last.x:.1f}, {last.y:.1f}) m')
        # Ready-made sim launch arguments: first route point, facing along the
        # first segment (the route always has at least 2 points).
        p0, p1 = response.path.poses[0].pose.position, response.path.poses[1].pose.position
        yaw_deg = math.degrees(math.atan2(p1.y - p0.y, p1.x - p0.x))
        print(f'Sim start on this route: start_x:={p0.x:.2f} start_y:={p0.y:.2f} '
              f'start_yaw_deg:={yaw_deg:.1f}')
        if args.save:
            path = os.path.abspath(os.path.expanduser(args.save))
            count = save_route_yaml(path, response)
            print(f'Saved {count} waypoints to {path}')
        return 0
    finally:
        node.destroy_node()
        rclpy.try_shutdown()