



"""Simulated robot: stands in for the Pioneer 3-AT and Nav2 during development.


In this file we describe our robot simulator.
In __init__ we set up all our parameters (the origin point, the hz rates, the robot’s speed limits),
create the motion model, the lock, the publishers, the timers and the action server.
The physics tick moves the robot one step using the model, then packages its new position,
speed and heading into ROS messages (odometry and IMU) and publishes them.
The GPS tick does the same for GPS: it reads the robot’s position, converts it to lat/lon
and publishes it. execute_goal takes each goal, hands it to the model, and then supervises it:
it keeps checking whether the goal has been reached, cancelled or replaced by a newer one,
and sends progress feedback until it finishes.
The model is what actually drives the robot towards the goal.
main turns everything on: it starts ROS, creates our node,
and runs an executor that calls our timers and execute_goal whenever they’re due,
until we hit Ctrl+C, then shuts everything down cleanly.



Offers a fake `navigate_to_pose` action (same interface as Nav2) and publishes
perfect GPS, IMU and odometry. Goals are in the `map` frame: metres east (x)
and north (y) of the origin.

How it fits together:
- A physics timer (20 Hz) moves the simulated robot and publishes odometry + IMU.
- A GPS timer (5 Hz) publishes the robot's position as latitude/longitude.
- An action server pretends to be Nav2: it accepts goals, drives the robot to
  them and reports progress, so other nodes can be developed without the real robot.
"""
import threading
import time

# ROS message and action types. Each ROS package generates Python classes from its
# .msg/.action files; importing them here lets us create and fill those messages.
from geometry_msgs.msg import Quaternion     # orientation (rotation) as 4 numbers
from nav2_msgs.action import NavigateToPose  # Nav2's "drive to this pose" action
from nav_msgs.msg import Odometry            # robot position + velocity from wheel motion
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus  # IMU reading, GPS fix, GPS status codes

# rclpy is the ROS Python library: everything needed to create nodes,
# publishers, timers and action servers, and to run them.
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

# Our own ROS-free code from guide_core (unit-tested separately).
from guide_core.angles import yaw_to_quaternion
from guide_core.geo import enu_to_latlon
from guide_core.sim_model import SimRobot


def quaternion_msg(yaw):
    """Build a ROS Quaternion message from a heading angle.

    ROS messages store orientation as a quaternion (x, y, z, w) rather than an
    angle. guide_core gives us the four numbers; this wraps them in the ROS
    message type. It lives here, not in guide_core, because it uses a ROS type.
    """
    q = Quaternion()
    q.x, q.y, q.z, q.w = yaw_to_quaternion(yaw)
    return q


class SimRobotNode(Node):
    # A ROS node in Python is a class that inherits from rclpy's Node. Inheriting
    # gives us all of Node's methods (create_publisher, create_timer,
    # declare_parameter, get_logger, get_clock...), which we call on `self`.

    def __init__(self):
        # Run Node's own setup first: registers this node with ROS under the
        # name 'sim_robot' (the name shown by `ros2 node list`).
        super().__init__('sim_robot')

        # --- Parameters ---
        # Settings that can be changed when the node is started, without editing
        # code (e.g. `ros2 run guide_sim sim_robot --ros-args -p max_speed:=1.5`).
        # declare_parameter(name, default) registers the setting; .value reads its
        # current value. ROS fixes each parameter's type from its default, so
        # defaults are written as floats (20.0, not 20).
        self.origin_lat = self.declare_parameter('origin_lat', 51.4988).value
        self.origin_lon = self.declare_parameter('origin_lon', -0.1749).value
        
        rate_hz = self.declare_parameter('rate_hz', 20.0).value
        gps_rate_hz = self.declare_parameter('gps_rate_hz', 5.0).value
        
        # The robot's motion model (pure Python, from guide_core). Its limits come
        # from parameters, so the simulated robot can be tuned from the command line.
        self.robot = SimRobot(
            max_speed=self.declare_parameter('max_speed', 1.0).value,
            max_turn_rate=self.declare_parameter('max_turn_rate', 1.0).value,
            goal_tolerance=self.declare_parameter('goal_tolerance', 0.3).value,
        )
        
        # Seconds of simulated time per physics step (0.05 s at 20 Hz).
        self.dt = 1.0 / rate_hz

        # The timers and the action server run in different threads, so every
        # access to self.robot and self.active_goal goes through this lock.
        self.lock = threading.Lock()
        
        # The goal currently being driven to. When a new goal arrives it replaces
        # this, and the older goal's loop notices and ends itself.
        self.active_goal = None

        # --- Publishers ---
        # Publishers send messages on a named topic. Any node subscribed to that
        # topic receives them. create_publisher(message_type, topic_name, qos):
        #   - topic names have no leading '/', so they can be namespaced later
        #   - qos sets delivery rules. qos_profile_sensor_data = "best effort, keep
        #     only the latest few": a lost sensor reading isn't resent because a
        #     newer one is coming. Subscribers to these topics must also use a
        #     best-effort profile, or they won't connect.
        #   - a plain number (10) means default reliable delivery, queue of 10.
        self.gps_pub = self.create_publisher(NavSatFix, 'gps/fix', qos_profile_sensor_data)
        self.imu_pub = self.create_publisher(Imu, 'imu/data', qos_profile_sensor_data)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)

        # --- Timers ---
        # create_timer(period_seconds, function) calls the function repeatedly.
        # Physics + odometry + IMU run fast (20 Hz); GPS runs slower (5 Hz), as a
        # real receiver does. The node keeps the timers alive internally, so we
        # don't need to store them. Note: no brackets after the function names,
        # we pass the function itself for the timer to call later.
        self.create_timer(self.dt, self.physics_tick)
        self.create_timer(1.0 / gps_rate_hz, self.publish_gps)

        # --- Fake Nav2 action server ---
        # An action is a long-running request: a client sends a goal, the server
        # works on it while sending feedback, and finishes with a result. We use
        # the same action type and name as real Nav2, so client code written
        # against this simulator works unchanged with the real robot.
        # ActionServer(node, action_type, action_name, ...):
        #   - execute_callback: function that carries out each accepted goal
        #   - goal_callback: decides whether to accept a new goal (we always accept)
        #   - cancel_callback: decides whether to allow cancelling (we always allow)
        #   - callback_group=ReentrantCallbackGroup(): lets several goals run at
        #     the same time, so a new goal can start while the old one's loop is
        #     still running (needed for one goal to replace another).
        # `lambda x: value` is a one-line function that ignores x and returns value.
        self.action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self.execute_goal,
            goal_callback=lambda goal: GoalResponse.ACCEPT,
            cancel_callback=lambda goal_handle: CancelResponse.ACCEPT,
            callback_group=ReentrantCallbackGroup(),
        )

        # get_logger() prints timestamped messages tagged with the node name.
        # Prefer it over print(): output can be filtered by severity and recorded.
        self.get_logger().info(
            f'Simulated robot ready, origin ({self.origin_lat}, {self.origin_lon})')

    # ------------------------------------------------------------------
    # Simulation and sensors
    # ------------------------------------------------------------------

    def physics_tick(self):
        """Advance the robot model one step, then publish odometry and IMU.

        Called by the physics timer 20 times a second.
        """
        # Move the robot one step and copy out its state. The copy is taken inside
        # the lock so the values are consistent; building messages happens after
        # releasing it, so other threads aren't kept waiting.
        with self.lock:
            self.robot.step(self.dt)
            s = self.robot.state
            x, y, yaw, v, omega = s.x, s.y, s.yaw, s.v, s.omega

        # Current ROS time, converted to the format message headers use. Both
        # messages describe the same instant, so they share one timestamp.
        stamp = self.get_clock().now().to_msg()

        # Odometry: where the robot is and how fast it's moving, as wheel encoders
        # would report it. Message structure:
        #   header.frame_id = 'odom'      -> position is measured in the odom frame
        #                                    (relative to where the robot started)
        #   child_frame_id = 'base_link'  -> it's the position of the robot's body
        #   pose.pose                     -> position + orientation (the outer
        #                                    'pose' also holds its uncertainty)
        #   twist.twist                   -> velocity: linear.x = forward speed (m/s),
        #                                    angular.z = turning rate (rad/s)
        # In this sim the robot starts at the map origin, so odom and map coincide.
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.orientation = quaternion_msg(yaw)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = omega
        self.odom_pub.publish(odom)

        # IMU: the robot's orientation and turning rate, as a motion sensor chip
        # would report it. Its frame is base_link because it's mounted on the robot.
        # We don't simulate acceleration; by ROS convention, setting the first
        # covariance entry to -1 tells consumers "this field isn't provided" so
        # they ignore it instead of trusting a zero.
        imu = Imu()
        imu.header.stamp = stamp
        imu.header.frame_id = 'base_link'
        imu.orientation = quaternion_msg(yaw)
        imu.angular_velocity.z = omega
        imu.linear_acceleration_covariance[0] = -1.0  # -1 means "not provided"
        self.imu_pub.publish(imu)

    def publish_gps(self):
        """Publish the robot's position as a GPS fix.

        Called by the GPS timer 5 times a second. Converts the robot's position
        in metres into latitude/longitude, as a real GPS receiver would output.
        """
        with self.lock:
            x, y = self.robot.state.x, self.robot.state.y
        lat, lon = enu_to_latlon(x, y, self.origin_lat, self.origin_lon)

        # NavSatFix is ROS's standard GPS message. Fields used:
        #   header.frame_id = 'gps_link' -> the GPS antenna's frame on the robot
        #   status.status = STATUS_FIX   -> "I have a valid position"
        #                                   (STATUS_NO_FIX = lost satellites)
        #   status.service = SERVICE_GPS -> which satellite system is used
        #   position_covariance_type     -> how much accuracy info is given;
        #                                   UNKNOWN = none (our GPS is perfect for
        #                                   now; noise is added in Step 8)
        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = 'gps_link'
        fix.status.status = NavSatStatus.STATUS_FIX
        fix.status.service = NavSatStatus.SERVICE_GPS
        fix.latitude = lat
        fix.longitude = lon
        fix.altitude = 0.0
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_UNKNOWN
        self.gps_pub.publish(fix)

    # ------------------------------------------------------------------
    # Fake Nav2
    # ------------------------------------------------------------------

    def execute_goal(self, goal_handle):
        """Drive to one goal, sending feedback, until reached, cancelled or replaced.

        Called by the action server once for each accepted goal, in its own thread.
        `goal_handle` represents that one goal: it holds the request and has
        methods to send feedback and to finish the goal with a status.
        """
        # goal_handle.request is the goal part of the NavigateToPose action:
        # a PoseStamped, i.e. a position + orientation tagged with a frame.
        target = goal_handle.request.pose
        
        # Goals must be in the map frame (metres east/north of the origin). A goal
        # in another frame (e.g. base_link = relative to the robot) would mean a
        # different place, so warn rather than silently misinterpret it.
        if target.header.frame_id not in ('', 'map'):
            self.get_logger().warn(
                f"Goal is in frame '{target.header.frame_id}', treating it as 'map'")
            
        gx, gy = target.pose.position.x, target.pose.position.y
        self.get_logger().info(f'New goal: x={gx:.1f} m, y={gy:.1f} m')

        # Take control of the robot. Pointing active_goal at this goal also tells
        # any older goal still running that it has been replaced.
        with self.lock:
            self.active_goal = goal_handle  # a newer goal replaces any older one
            self.robot.set_goal(gx, gy)

        # Remember the start time for the feedback's navigation_time field, and
        # create one Feedback message (fields defined by the action) to reuse.
        start = self.get_clock().now()
        feedback = NavigateToPose.Feedback()

        # Check progress 10 times a second until the goal ends.
        # rclpy.ok() stays True until ROS is shut down (e.g. Ctrl+C).
        while rclpy.ok():
            # Decide what's happening, in order of priority: replaced by a newer
            # goal, cancel requested by the client, arrived, or still driving.
            # goal_handle.is_cancel_requested is set by ROS when the client asks
            # to cancel. Cancelled/reached goals release the robot; a replaced goal
            # must not, because the robot now belongs to the newer goal. All of
            # this happens inside one lock so a new goal can't slip in between
            # the check and the release.
            with self.lock:
                if self.active_goal is not goal_handle:
                    outcome = 'replaced'
                elif goal_handle.is_cancel_requested:
                    outcome = 'cancelled'
                elif self.robot.goal_reached():
                    outcome = 'reached'
                else:
                    outcome = None
                if outcome in ('cancelled', 'reached'):
                    self.robot.clear_goal()
                    self.active_goal = None
                distance = self.robot.distance_to_goal()
                s = self.robot.state
                x, y, yaw = s.x, s.y, s.yaw

            # End the goal if needed. The status call tells the client how the goal
            # ended (this is the main answer it receives):
            #   abort()    -> failed / stopped by the server
            #   canceled() -> stopped because the client asked
            #   succeed()  -> completed
            # Returning a Result message is required; its error_code defaults to 0
            # ("no error").
            if outcome == 'replaced':
                self.get_logger().info('Goal replaced by a newer one')
                goal_handle.abort()
                return NavigateToPose.Result()
            if outcome == 'cancelled':
                self.get_logger().info('Goal cancelled')
                goal_handle.canceled()
                return NavigateToPose.Result()
            if outcome == 'reached':
                self.get_logger().info('Goal reached')
                goal_handle.succeed()
                return NavigateToPose.Result()

            # Still driving: send progress to the client. These are fields of the
            # NavigateToPose feedback definition (see `ros2 interface show
            # nav2_msgs/action/NavigateToPose`). Subtracting two ROS times gives a
            # Duration, which .to_msg() converts for the message.
            # publish_feedback() sends it to the client that sent this goal.
            now = self.get_clock().now()
            feedback.current_pose.header.stamp = now.to_msg()
            feedback.current_pose.header.frame_id = 'map'
            feedback.current_pose.pose.position.x = x
            feedback.current_pose.pose.position.y = y
            feedback.current_pose.pose.orientation = quaternion_msg(yaw)
            feedback.distance_remaining = distance
            feedback.navigation_time = (now - start).to_msg()
            goal_handle.publish_feedback(feedback)

            # Wait before checking again. This only pauses this goal's thread;
            # the timers keep running in other threads (see main()).
            time.sleep(0.1)  # send feedback 10 times a second

        # Only reached if ROS shuts down mid-goal: end it as failed.
        goal_handle.abort()
        return NavigateToPose.Result()


def main(args=None):
    """Entry point: start ROS, run the node until Ctrl+C, then clean up.

    `ros2 run guide_sim sim_robot` calls this function (registered in setup.py).
    """
    # Start the ROS library (also reads any --ros-args, e.g. parameter
    # overrides), then create our node, which runs __init__ above.
    rclpy.init(args=args)
    node = SimRobotNode()

    # The executor waits for events (a timer is due, a goal arrives) and calls the
    # matching function. A MultiThreadedExecutor can run several at once, which
    # we need: execute_goal loops for as long as the robot drives, and the
    # physics timer must keep running at the same time. A single-threaded
    # executor would freeze the simulation during every goal.
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    # spin() runs until Ctrl+C (KeyboardInterrupt) or ROS is shut down from
    # outside (ExternalShutdownException). Either way, `finally` destroys the
    # node and shuts ROS down cleanly; try_shutdown() doesn't complain if ROS
    # is already shut down.
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


# Allows running the file directly with `python3 sim_robot_node.py`.
# Normally it's started with `ros2 run`, which calls main() itself.
if __name__ == '__main__':
    main()