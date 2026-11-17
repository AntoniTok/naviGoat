"""Localizer v1 node: GPS + IMU heading + SLAM pose -> robot pose in ENU (Step 8).

Subscribes to the GPS fix (NavSatFix, parameter gps_topic, default gps/fix)
and the IMU (Imu, parameter imu_topic, default imu/data), and reads the robot's
SLAM pose from TF (map -> base_link: RTAB-Map's map -> odom followed by the
base driver's odom -> base_link; on the simulator, sim_robot publishes both).
The alignment filter (guide_core.alignment) estimates T = where the map frame
sits in the ENU frame around the configured origin; the robot's ENU pose is T
composed with its SLAM pose: smooth like SLAM, anchored to the world like GPS.

Publishes:
- guide/pose (PoseWithCovarianceStamped, 10 Hz): the robot in ENU, with the
  filter's covariance. Nothing until the filter is initialised.
- guide/map_in_enu (TransformStamped, 10 Hz): T. Goals for Nav2 are converted
  from ENU into the map frame with it (Step 8.8).
- guide/localization_status (LocalizationStatus, 2 Hz): GOOD / DEGRADED / LOST
  (guide_core.localization_quality), sigma, reason, fix counters.

This node never broadcasts TF (RTAB-Map owns map -> odom). The maths lives in
guide_core; this node only moves data between ROS messages and those modules.
"""

from dataclasses import fields
import math

# Message types. Each ROS message is a Python class with typed fields.
from geometry_msgs.msg import PoseWithCovarianceStamped, TransformStamped  # our outputs
from guide_core.alignment import AlignmentConfig, AlignmentFilter, FixResult
from guide_core.angles import quaternion_to_yaw, yaw_to_quaternion
from guide_core.localization_quality import QualityConfig, QualityMonitor
from guide_core.localizer import (
    GpsFix, GpsLocalizer, InvalidFixError, pose_covariance_6x6, PositionEstimate)
from guide_core.se2 import Pose2D
from guide_interfaces.msg import LocalizationStatus
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# Sensor-data QoS: "best effort" delivery that keeps only the newest few
# messages. Old sensor readings are useless, so dropping one is better than
# waiting for it to be resent. Must match what the sensor publishes with.
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import Imu, NavSatFix               # our inputs
# tf2_ros keeps a buffer of recent transforms (TF) received on /tf, so we can ask
# "where was base_link in the map frame at time t?".
from tf2_ros import Buffer, TransformException, TransformListener


def declare_config(node, prefix, config_class):
    """Declare one ROS parameter per field of a config dataclass and build it.

    E.g. AlignmentConfig.gps_gate becomes the parameter 'alignment.gps_gate', so any
    tuning value can be set at launch (-p alignment.gps_gate:=9.0) or from a
    parameter file, without adding code here for each one. Defaults come from
    the dataclass; ROS fixes each parameter's type from its default.
    """
    values = {}
    for f in fields(config_class):
        values[f.name] = node.declare_parameter(f'{prefix}.{f.name}', f.default).value
    return config_class(**values)


def stamp_seconds(stamp):
    """Return a message header stamp as float seconds."""
    return stamp.sec + stamp.nanosec * 1e-9


class LocalizerNode(Node):
    """Thin ROS wrapper around guide_core's AlignmentFilter and QualityMonitor."""

    def __init__(self):
        # 'localizer' is the node's name, as shown by `ros2 node list`.
        super().__init__('localizer')

        # --- Parameters ---
        # Settings that can be changed at launch without editing code, e.g.
        #   ros2 run guide_nodes localizer --ros-args -p origin_lat:=51.5
        # declare_parameter(name, default) registers the parameter;
        # .value reads whatever was actually set (or the default).
        # The origin MUST match the simulator's / the route's origin, or every
        # position will be offset.
        origin_lat = self.declare_parameter('origin_lat', 51.4988).value
        origin_lon = self.declare_parameter('origin_lon', -0.1749).value
        # Variance (m^2) used when the GPS doesn't report its accuracy.
        # 25 m^2 = standard deviation of 5 m.
        default_gps_variance = self.declare_parameter('default_gps_variance', 25.0).value
        # TF frames: the SLAM map frame and the robot's body frame.
        self.map_frame = self.declare_parameter('map_frame', 'map').value
        self.base_frame = self.declare_parameter('base_frame', 'base_link').value
        # frame_id written on guide/pose: the pose is in ENU, the frame of the route.
        self.pose_frame = self.declare_parameter('pose_frame', 'enu').value
        # frame_id for ENU on guide/map_in_enu.
        self.enu_frame = self.declare_parameter('enu_frame', 'enu').value
        # Sensor topics (Step 10). Set in config/gps.yaml to the names the robot's
        # GPS and IMU drivers publish on. A name without a leading '/' is relative,
        # so it can be namespaced or remapped at launch.
        gps_topic = self.declare_parameter('gps_topic', 'gps/fix').value
        imu_topic = self.declare_parameter('imu_topic', 'imu/data').value
        pose_rate = self.declare_parameter('pose_rate', 10.0).value
        status_rate = self.declare_parameter('status_rate', 2.0).value

        # --- Core logic (pure Python, unit-tested in guide_core) ---
        # GpsLocalizer (v0) validates fixes and converts lat/lon to ENU metres.
        # Its constructor raises ValueError on a bad origin, which stops the node
        # at startup with a clear message instead of publishing nonsense later.
        self.converter = GpsLocalizer(origin_lat, origin_lon,
                                      default_variance=default_gps_variance)
        self.filter = AlignmentFilter(declare_config(self, 'alignment', AlignmentConfig))
        self.quality = QualityMonitor(declare_config(self, 'quality', QualityConfig))
        self.fixes_accepted = 0
        self.fixes_rejected = 0

        # --- TF listener ---
        # The Buffer stores transforms; the TransformListener subscribes to /tf
        # and /tf_static and fills the buffer. Keep a reference to the listener,
        # or it is garbage-collected and stops listening.
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # --- Publishers ---
        # create_publisher(type, topic, qos). The integer 10 is shorthand for
        # the default QoS: "reliable" delivery, queue depth 10. Reliable suits
        # outputs that other nodes act on; the sensor-data QoS is only for raw
        # sensor topics. The status changes every half second, so it is not latched.
        self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, 'guide/pose', 10)
        self.map_pub = self.create_publisher(TransformStamped, 'guide/map_in_enu', 10)
        self.status_pub = self.create_publisher(
            LocalizationStatus, 'guide/localization_status', 10)

        # --- Subscribers ---
        # create_subscription(type, topic, callback, qos): every time a
        # message arrives on the topic, ROS calls the callback with it.
        # The topic names come from the parameters above.
        self.gps_sub = self.create_subscription(
            NavSatFix, gps_topic, self.on_gps_fix, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self.on_imu, qos_profile_sensor_data)

        # --- Timers ---
        # The pose is published at a steady rate from the latest SLAM pose (not
        # once per GPS fix as in v0), so it stays smooth and keeps coming in tunnels.
        self.create_timer(1.0 / pose_rate, self.publish_pose)
        self.create_timer(1.0 / status_rate, self.publish_status)

        self.get_logger().info(
            f'Localizer v1 started: origin=({origin_lat}, {origin_lon}), GPS on {gps_topic}, '
            f'IMU on {imu_topic}, SLAM pose from TF {self.map_frame} -> {self.base_frame}, '
            f'publishing guide/pose (frame "{self.pose_frame}")')

    # --- Helpers ---

    def now_seconds(self):
        """Return the node clock's current time in seconds."""
        return self.get_clock().now().nanoseconds * 1e-9

    def map_pose(self, stamp=None):
        """Return the robot's SLAM pose (map -> base_link) as a Pose2D, or None.

        stamp: the time wanted (a message header stamp), or None for the latest.
        If TF has nothing for that exact time yet (the newest transform is slightly
        older than the fix), the latest pose is used instead.
        """
        times = [Time()] if stamp is None else [Time.from_msg(stamp), Time()]
        for when in times:
            try:
                # Time() (zero) means "the latest available transform".
                t = self.tf_buffer.lookup_transform(self.map_frame, self.base_frame, when)
            except TransformException:
                continue
            q = t.transform.rotation
            return Pose2D(t.transform.translation.x, t.transform.translation.y,
                          quaternion_to_yaw(q.x, q.y, q.z, q.w))
        self.get_logger().warning(
            f'No TF {self.map_frame} -> {self.base_frame} yet (is the SLAM / sim running?)',
            throttle_duration_sec=5.0)
        return None

    # --- IMU callback: absolute heading for the filter ---
    # The IMU gives orientation as a quaternion that may include tilt (roll,
    # pitch). Our system is 2D, so we keep only the yaw (compass direction).
    def on_imu(self, msg: Imu):
        # ROS convention (see sensor_msgs/Imu): orientation_covariance[0] == -1
        # means "this IMU does not provide orientation at all".
        if msg.orientation_covariance[0] == -1.0:
            self.get_logger().warning(
                'IMU does not provide orientation; heading comes from GPS only',
                throttle_duration_sec=30.0)
            return
        pose = self.map_pose()
        if pose is None:
            return
        q = msg.orientation
        self.filter.heading(quaternion_to_yaw(q.x, q.y, q.z, q.w), pose)

    # --- GPS callback: one fix for the filter ---
    def on_gps_fix(self, msg: NavSatFix):
        # Copy the fields we need into guide_core's ROS-free GpsFix.
        # msg.status.status is the fix status (-1 = no fix);
        # msg.position_covariance arrives as a numpy array, so it is
        # copied into a tuple, which can't be changed afterwards.
        fix = GpsFix(
            latitude=msg.latitude,
            longitude=msg.longitude,
            status=msg.status.status,
            position_covariance=tuple(msg.position_covariance),
            position_covariance_type=msg.position_covariance_type,
        )
        try:
            estimate = self.converter.convert(fix)
        except InvalidFixError as err:
            # No fix (e.g. a tunnel) is normal, so this is only debug output;
            # the status topic reports "no GPS fix for N s".
            self.get_logger().debug(f'GPS fix not used: {err}')
            return
        # The SLAM pose at the moment the fix was MEASURED (its header stamp).
        pose = self.map_pose(msg.header.stamp)
        if pose is None:
            return
        sigma = math.sqrt(max(estimate.var_x, estimate.var_y))
        update = self.filter.gps(stamp_seconds(msg.header.stamp), estimate.x, estimate.y,
                                 sigma, pose)
        if update.result in (FixResult.ACCEPTED, FixResult.REACQUIRED):
            self.fixes_accepted += 1
        elif update.result is FixResult.REJECTED:
            self.fixes_rejected += 1
            self.get_logger().info(
                f'GPS fix rejected as an outlier ({update.distance:.1f} sigma)',
                throttle_duration_sec=5.0)
        if update.result in (FixResult.INITIALISED, FixResult.REACQUIRED):
            self.get_logger().info(f'Alignment {update.result.value}: map frame in ENU at '
                                   f'({self.filter.transform.x:.1f}, '
                                   f'{self.filter.transform.y:.1f}) m, '
                                   f'{math.degrees(self.filter.transform.yaw):.1f} deg')

    # --- Pose timer: robot pose in ENU and the map frame in ENU ---
    def publish_pose(self):
        pose = self.map_pose()
        if pose is None:
            return
        # Let the filter account for the drift since the previous call.
        self.filter.predict(pose, self.now_seconds())
        robot = self.filter.robot_pose(pose)
        if robot is None:
            return      # not initialised yet: no pose rather than a wrong one
        stamp = self.get_clock().now().to_msg()

        # PoseWithCovarianceStamped: position + orientation + a 6x6 covariance
        # (x, y, z, roll, pitch, yaw); pose_covariance_6x6 fills it from the filter.
        cov = self.filter.position_covariance(pose)
        out = PoseWithCovarianceStamped()
        out.header.stamp = stamp
        out.header.frame_id = self.pose_frame
        out.pose.pose.position.x = robot.x
        out.pose.pose.position.y = robot.y
        qx, qy, qz, qw = yaw_to_quaternion(robot.yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = pose_covariance_6x6(
            PositionEstimate(robot.x, robot.y, cov[0][0], cov[1][1], cov[0][1]),
            self.filter.yaw_sigma() ** 2)
        self.pose_pub.publish(out)
        self.get_logger().info('First pose published', once=True)

        # TransformStamped: "child frame (map) expressed in parent frame (enu)".
        # Published as a topic, NOT on /tf: RTAB-Map owns the TF tree.
        t = self.filter.transform
        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.enu_frame
        tf.child_frame_id = self.map_frame
        tf.transform.translation.x = t.x
        tf.transform.translation.y = t.y
        qx, qy, qz, qw = yaw_to_quaternion(t.yaw)
        tf.transform.rotation.x = qx
        tf.transform.rotation.y = qy
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.map_pub.publish(tf)

    # --- Status timer: GOOD / DEGRADED / LOST ---
    def publish_status(self):
        now = self.now_seconds()
        pose = self.map_pose()
        sigma = math.inf if pose is None else self.filter.position_sigma(pose)
        since_fix = self.filter.seconds_since_fix(now)
        state = self.quality.update(now, self.filter.initialised, sigma, since_fix)
        if state.changed:
            self.get_logger().info(f'Localization {state.quality.name}: {state.reason}')

        msg = LocalizationStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.pose_frame
        msg.quality = int(state.quality)
        msg.reason = state.reason
        msg.initialised = self.filter.initialised
        msg.position_sigma = sigma
        msg.yaw_sigma = self.filter.yaw_sigma()
        msg.seconds_since_fix = since_fix
        msg.distance_since_fix = self.filter.distance_since_fix
        msg.fixes_accepted = self.fixes_accepted
        msg.fixes_rejected = self.fixes_rejected
        self.status_pub.publish(msg)


# --- Entry point ---
# rclpy.init() starts ROS for this process; rclpy.spin() then waits for
# messages and runs callbacks until the node is stopped (Ctrl+C).
# The default executor is single-threaded: callbacks (GPS, IMU, timers, TF) run
# one at a time, so they never interrupt each other and no lock is needed.
def main(args=None):
    rclpy.init(args=args)
    node = LocalizerNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C or shutdown from outside: exit quietly
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
