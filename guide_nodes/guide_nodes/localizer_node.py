"""Localizer v0 node: GPS position + IMU heading -> robot pose in the map frame.

Subscribes to gps/fix (NavSatFix) and imu/data (Imu), and publishes guide/pose
(PoseWithCovarianceStamped), one pose per valid GPS fix. No filtering.
The maths lives in guide_core.localizer; this node only moves data between
ROS messages and that module.

v0 assumption: the map frame IS the east-north frame around the configured
origin (x = east, y = north). True in the simulator; real map/GPS alignment
comes in Step 8.
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
# Sensor-data QoS: "best effort" delivery that keeps only the newest few
# messages. Old sensor readings are useless, so dropping one is better than
# waiting for it to be resent. Must match what the sensor publishes with.
from rclpy.qos import qos_profile_sensor_data

# Message types. Each ROS message is a Python class with typed fields.
from geometry_msgs.msg import PoseWithCovarianceStamped  # our output
from sensor_msgs.msg import Imu, NavSatFix               # our inputs

from guide_core.angles import quaternion_to_yaw, yaw_to_quaternion
from guide_core.localizer import (
    GpsFix,
    GpsLocalizer,
    InvalidFixError,
    pose_covariance_6x6,
)


class LocalizerNode(Node):
    """Thin ROS wrapper around guide_core.localizer.GpsLocalizer."""

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
        self.declare_parameter('origin_lat', 51.4988)
        self.declare_parameter('origin_lon', -0.1749)
        # Variance (m^2) used when the GPS doesn't report its accuracy.
        # 25 m^2 = standard deviation of 5 m.
        self.declare_parameter('default_gps_variance', 25.0)
        # Variance (rad^2) used when the IMU doesn't report heading accuracy.
        # (5 degrees)^2, roughly 0.0076 rad^2.
        self.declare_parameter('default_yaw_variance', math.radians(5.0) ** 2)
        # Name of the frame our poses are expressed in.
        self.declare_parameter('map_frame', 'map')

        origin_lat = self.get_parameter('origin_lat').value
        origin_lon = self.get_parameter('origin_lon').value
        default_gps_variance = self.get_parameter('default_gps_variance').value
        self.default_yaw_variance = self.get_parameter('default_yaw_variance').value
        self.map_frame = self.get_parameter('map_frame').value

        # --- Core logic (pure Python, unit-tested in guide_core) ---
        # Raises ValueError on a bad origin, which stops the node at startup
        # with a clear message instead of publishing nonsense later.
        self.localizer = GpsLocalizer(origin_lat, origin_lon,
                                      default_variance=default_gps_variance)

        # --- Latest heading from the IMU ---
        # IMU messages only update these; poses are published on GPS fixes.
        # None means "no heading received yet", so don't publish.
        self.latest_yaw = None
        self.latest_yaw_variance = self.default_yaw_variance

        # --- Publisher ---
        # create_publisher(type, topic, qos). The integer 10 is shorthand for
        # the default QoS: "reliable" delivery, queue depth 10. Reliable suits
        # an output that other nodes act on; the sensor-data QoS is only for
        # raw sensor topics.
        self.pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, 'guide/pose', 10)

        # --- Subscribers ---
        # create_subscription(type, topic, callback, qos): every time a
        # message arrives on the topic, ROS calls the callback with it.
        # Topic names without a leading '/' are relative, so they can be
        # remapped or namespaced at launch.
        self.gps_sub = self.create_subscription(
            NavSatFix, 'gps/fix', self.on_gps_fix, qos_profile_sensor_data)
        self.imu_sub = self.create_subscription(
            Imu, 'imu/data', self.on_imu, qos_profile_sensor_data)

        self.get_logger().info(
            f'Localizer v0 started: origin=({origin_lat}, {origin_lon}), '
            f'publishing guide/pose in frame "{self.map_frame}"')

    # --- IMU callback: remember the latest heading ---
    # The IMU gives orientation as a quaternion that may include tilt (roll,
    # pitch). Our system is 2D, so we keep only the yaw (compass direction).
    def on_imu(self, msg: Imu):
        # ROS convention (see sensor_msgs/Imu): orientation_covariance[0] == -1
        # means "this IMU does not provide orientation at all".
        if msg.orientation_covariance[0] == -1.0:
            self.get_logger().warning(
                'IMU does not provide orientation; no heading available',
                throttle_duration_sec=10.0)
            return

        q = msg.orientation
        self.latest_yaw = quaternion_to_yaw(q.x, q.y, q.z, q.w)

        # orientation_covariance is a 3x3 grid (roll, pitch, yaw), row-major,
        # so index 8 is the yaw variance. 0 or junk means "unknown".
        yaw_var = msg.orientation_covariance[8]
        if math.isfinite(yaw_var) and yaw_var > 0.0:
            self.latest_yaw_variance = yaw_var
        else:
            self.latest_yaw_variance = self.default_yaw_variance

    # --- GPS callback: convert the fix and publish a pose ---
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
            estimate = self.localizer.convert(fix)
        except InvalidFixError as err:
            # throttle_duration_sec: print at most once every 5 s so a run of
            # bad fixes (5 per second) doesn't flood the terminal.
            self.get_logger().warning(f'GPS fix rejected: {err}',
                                      throttle_duration_sec=5.0)
            return

        if self.latest_yaw is None:
            self.get_logger().warning(
                'Waiting for IMU heading before publishing poses',
                throttle_duration_sec=5.0)
            return

        # Build the output message.
        # header.stamp: copied from the GPS message, i.e. when the position
        #   was MEASURED, not when we happened to process it.
        # header.frame_id: which coordinate system x/y are in.
        # pose.pose: position + orientation; pose.covariance: 36 numbers
        #   (6x6 grid over x, y, z, roll, pitch, yaw).
        out = PoseWithCovarianceStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame

        out.pose.pose.position.x = estimate.x
        out.pose.pose.position.y = estimate.y
        out.pose.pose.position.z = 0.0

        qx, qy, qz, qw = yaw_to_quaternion(self.latest_yaw)
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw

        out.pose.covariance = pose_covariance_6x6(estimate,
                                                  self.latest_yaw_variance)

        self.pose_pub.publish(out)
        # once=True: log only the very first time, as confirmation.
        self.get_logger().info('First pose published', once=True)


# --- Entry point ---
# rclpy.init() starts ROS for this process; rclpy.spin() then waits for
# messages and runs callbacks until the node is stopped (Ctrl+C).
# The default executor is single-threaded: callbacks run one at a time, so
# on_imu and on_gps_fix can never run in the middle of each other, and no
# lock is needed (unlike the multithreaded simulator).
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