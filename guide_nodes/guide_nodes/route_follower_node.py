"""Route follower node: follows a route and chooses goals for Nav2.

Loads a route from a YAML file, projects each guide/pose onto it and
publishes progress (guide/route_progress), the route itself (guide/route)
and the next goal for the goal sender (guide/next_goal). All geometry is
done in guide_core.route_follower; this node only converts messages.
"""

import math

import rclpy
import yaml
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from rclpy.executors import ExternalShutdownException
from rclpy.logging import get_logger
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from guide_core.angles import yaw_to_quaternion
from guide_core.route import InvalidRouteError, route_from_dict
from guide_core.route_follower import FollowerConfig, Goal, RouteFollower
from guide_interfaces.msg import NavGoalStatus, RouteProgress

# "Latched" QoS for topics that describe current state and change rarely.
# RELIABLE: messages are re-sent until received (no silent drops).
# TRANSIENT_LOCAL: the publisher keeps its last message and delivers it to
# subscribers that connect later (e.g. RViz started after this node).
# depth=1: only the latest message is kept. A subscriber must also request
# TRANSIENT_LOCAL to receive the stored message.
LATCHED_QOS = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


class RouteLoadError(Exception):
    """The route file or the follower parameters could not be used."""


def set_yaw(pose_msg, yaw: float) -> None:
    """Write a heading into a geometry_msgs/Pose orientation (a quaternion)."""
    # ROS stores orientation as a quaternion (x, y, z, w); for a pure
    # heading only z and w are non-zero.
    q = yaw_to_quaternion(yaw)
    pose_msg.orientation.x, pose_msg.orientation.y, \
        pose_msg.orientation.z, pose_msg.orientation.w = q


class RouteFollowerNode(Node):
    """ROS wrapper around guide_core.route_follower.RouteFollower."""

    def __init__(self):
        super().__init__('route_follower')

        # Parameters. declare_parameter fixes each parameter's name, type and
        # default; values can be overridden with --ros-args -p name:=value.
        # Floats must be given as floats (20.0, not 20). Distance defaults
        # come from FollowerConfig so they are defined in one place.
        defaults = FollowerConfig()
        self.declare_parameter('route_file', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('lookahead', defaults.lookahead)
        self.declare_parameter('corner_angle_deg', 30.0)  # degrees, for humans
        self.declare_parameter('corner_release_distance', defaults.corner_release_distance)
        self.declare_parameter('goal_update_distance', defaults.goal_update_distance)
        self.declare_parameter('arrival_tolerance', defaults.arrival_tolerance)
        self.declare_parameter('window_behind', defaults.window_behind)
        self.declare_parameter('window_ahead', defaults.window_ahead)

        self._map_frame = self.get_parameter('map_frame').value
        route_file = self.get_parameter('route_file').value

        # Route and follower. Any problem becomes a RouteLoadError so main()
        # can stop the node with one clear message.
        route = self._load_route(route_file)
        try:
            config = FollowerConfig(
                lookahead=self.get_parameter('lookahead').value,
                corner_angle=math.radians(self.get_parameter('corner_angle_deg').value),
                corner_release_distance=self.get_parameter('corner_release_distance').value,
                goal_update_distance=self.get_parameter('goal_update_distance').value,
                arrival_tolerance=self.get_parameter('arrival_tolerance').value,
                window_behind=self.get_parameter('window_behind').value,
                window_ahead=self.get_parameter('window_ahead').value,
            )
        except ValueError as e:
            raise RouteLoadError(f'invalid follower parameter: {e}') from e
        self._follower = RouteFollower(route, config)
        self._arrival_logged = False
        
        # Stamp of the last goal published on guide/next_goal. The goal sender
        # copies it into guide/goal_status, so it tells us which goal a status
        # is about (see _on_goal_status). None until the first goal.
        self._last_goal_stamp = None
        # Robot's s from the latest pose, for log messages only.
        self._last_robot_s = 0.0

        # Publishers. Route and next goal are state, so latched; progress is
        # a 5 Hz stream, so default reliable QoS with a queue of 10.
        self._route_pub = self.create_publisher(Path, 'guide/route', LATCHED_QOS)
        self._goal_pub = self.create_publisher(PoseStamped, 'guide/next_goal', LATCHED_QOS)
        self._progress_pub = self.create_publisher(RouteProgress, 'guide/route_progress', 10)

        # Subscriber to our own localizer's pose. It is published with default
        # reliable QoS, so we subscribe with the default too (depth 10).
        self.create_subscription(
            PoseWithCovarianceStamped, 'guide/pose', self._on_pose, 10)
        
        
        # Subscriber to the goal sender's status (corner safety net). It is
        # published latched, so we subscribe with the same LATCHED_QOS:
        # TRANSIENT_LOCAL so we still get the latest status if we start later,
        # RELIABLE so no status change is silently dropped.
        self.create_subscription(
            NavGoalStatus, 'guide/goal_status', self._on_goal_status, LATCHED_QOS)

        self._publish_route()
        self.get_logger().info(
            f'Loaded route {route_file}: {len(route.points)} waypoints, '
            f'{route.total_length:.1f} m, '
            f'{len(route.corners(config.corner_angle))} corner(s)')

    def _load_route(self, route_file: str):
        """Read and validate the route file; raise RouteLoadError on any problem."""
        if not route_file:
            raise RouteLoadError("parameter 'route_file' is not set")
        try:
            with open(route_file) as f:
                data = yaml.safe_load(f)
            return route_from_dict(data, expected_frame=self._map_frame)
        except OSError as e:
            raise RouteLoadError(f'cannot read route file {route_file!r}: {e}') from e
        except yaml.YAMLError as e:
            raise RouteLoadError(f'route file {route_file!r} is not valid YAML: {e}') from e
        except InvalidRouteError as e:
            raise RouteLoadError(f'invalid route in {route_file!r}: {e}') from e

    def _publish_route(self) -> None:
        """Publish the whole route once as a nav_msgs/Path (latched)."""
        # A Path is a header plus a list of PoseStamped. Each waypoint's
        # orientation is the direction of the segment leaving it.
        route = self._follower.route
        path = Path()
        path.header.frame_id = self._map_frame
        path.header.stamp = self.get_clock().now().to_msg()
        for s, (x, y) in zip(route.cumulative, route.points):
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            set_yaw(pose.pose, route.heading_at(s))
            path.poses.append(pose)
        self._route_pub.publish(path)

    def _on_pose(self, msg: PoseWithCovarianceStamped) -> None:
        """Update the follower with a new pose and publish the results."""
        # Only map-frame poses can be compared with the route; anything else
        # is a configuration error, reported at most every 5 s.
        if msg.header.frame_id != self._map_frame:
            self.get_logger().warning(
                f"ignoring pose in frame '{msg.header.frame_id}', "
                f"expected '{self._map_frame}'",
                throttle_duration_sec=5.0)
            return

        # PoseWithCovarianceStamped nests as msg.pose (with covariance)
        # .pose (the Pose) .position; only x and y are used.
        position = msg.pose.pose.position
        try:
            out = self._follower.update(position.x, position.y)
        except ValueError as e:
            self.get_logger().warning(f'ignoring pose: {e}', throttle_duration_sec=5.0)
            return
        
        self._last_robot_s = out.s

        # Progress on every pose. The stamp is copied from the pose, so each
        # output can be traced back to the measurement it came from.
        progress = RouteProgress()
        progress.header.stamp = msg.header.stamp
        progress.header.frame_id = self._map_frame
        progress.segment_index = out.segment_index
        progress.distance_along = out.s
        progress.distance_remaining = out.distance_remaining
        progress.total_length = out.total_length
        progress.cross_track_error = out.cross_track_error
        progress.arrived = out.arrived
        self._progress_pub.publish(progress)

        if out.new_goal is not None:
            self._publish_goal(out.new_goal, msg.header.stamp, out.s)

        if out.arrived and not self._arrival_logged:
            self.get_logger().info(
                f'Arrived: {out.distance_remaining:.2f} m remaining along the route')
            self._arrival_logged = True

    def _publish_goal(self, goal: Goal, stamp, robot_s: float) -> None:
        """Publish a new goal for the goal sender (latched)."""
        msg = PoseStamped()
        msg.header.stamp = stamp
        msg.header.frame_id = self._map_frame
        msg.pose.position.x = goal.x
        msg.pose.position.y = goal.y
        set_yaw(msg.pose, goal.yaw)
        self._goal_pub.publish(msg)
        self._last_goal_stamp = stamp

        kind = 'final' if goal.is_final else 'corner' if goal.is_corner else 'lookahead'
        self.get_logger().info(
            f'New goal ({kind}) at s={goal.s:.1f} m: x={goal.x:.1f} y={goal.y:.1f} '
            f'yaw={math.degrees(goal.yaw):.0f} deg (robot at s={robot_s:.1f} m)')


    
    def _on_goal_status(self, msg: NavGoalStatus) -> None:
        """Corner safety net: release a corner goal that Nav2 reports as reached.

        Nav2 stops ~0.25 m short of a goal and reports SUCCEEDED. If that
        happens before the robot is within corner_release_distance of a held
        corner, the robot would wait at the corner forever. Here a SUCCEEDED
        for our current corner goal releases it; the next pose then chooses a
        goal past the corner (goals are only ever chosen in _on_pose).
        """
        # Only successes can release a corner. PENDING/ACTIVE/ABORTED/... are
        # ignored here (the mission manager will use them in Step 7).
        if msg.status != NavGoalStatus.SUCCEEDED:
            return

        goal = msg.goal
        if goal.header.frame_id != self._map_frame:
            self.get_logger().warning(
                f"ignoring goal status in frame '{goal.header.frame_id}', "
                f"expected '{self._map_frame}'",
                throttle_duration_sec=5.0)
            return

        # Layer 1 against stale messages: the status must be about the goal we
        # published last. Each goal's stamp is unique (copied from a pose), and
        # the goal sender sends it back unchanged. ROS message objects compare
        # field by field with ==, so this compares sec and nanosec. A status
        # left over from an older goal, or from before this node started,
        # fails here.
        if self._last_goal_stamp is None or goal.header.stamp != self._last_goal_stamp:
            return

        # Layer 2 (guide_core): position matches the current goal, and it is
        # a corner that is not yet released. Read the goal first for the log.
        current = self._follower.goal
        position = goal.pose.position
        if self._follower.confirm_goal_reached(position.x, position.y):
            self.get_logger().info(
                f'Corner at s={current.s:.1f} m confirmed reached by Nav2: released '
                f'(robot at s={self._last_robot_s:.1f} m)')

def main(args=None):
    rclpy.init(args=args)

    # A node that cannot load its route exits with a FATAL log instead of
    # running and silently publishing nothing.
    try:
        node = RouteFollowerNode()
    except RouteLoadError as e:
        get_logger('route_follower').fatal(str(e))
        rclpy.shutdown()
        return 1

    # spin() runs callbacks until Ctrl-C. A single-threaded executor is
    # enough: the callbacks (pose, goal status) never block, and running one
    # at a time means they never change the follower at the same moment.
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0