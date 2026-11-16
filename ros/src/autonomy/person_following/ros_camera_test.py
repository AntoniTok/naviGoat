"""ROS camera-only test: subscribe to tracks and PRINT hypothetical velocities.

This runner creates no publishers and cannot send commands to a robot.
Enable using --enable-on-start or /person_following/set_enabled (SetBool).
Run --help on any laptop; ROS imports are deferred until running the live test.
"""

from __future__ import annotations

import argparse
import math
import time

from perception_adapter import PerceptionAdapter, PerceptionError
from state_machine import FollowingConfig, FollowingStateMachine
from velocity_controller import ControllerConfig, VelocityController


def positive_number(text):
    value = float(text)
    if not math.isfinite(value) or value <= 0:
        raise argparse.ArgumentTypeError("Value must be finite and positive")
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/social_perception/tracks")
    parser.add_argument("--enable-on-start", action="store_true")
    parser.add_argument("--following-distance", type=positive_number, default=1.0)
    parser.add_argument("--input-timeout", type=positive_number, default=0.5)
    parser.add_argument("--loss-timeout", type=positive_number, default=2.0)
    parser.add_argument("--control-hz", type=positive_number, default=20.0)
    args, ros_args = parser.parse_known_args()
    try:
        controller = VelocityController(ControllerConfig(
            following_distance_m=args.following_distance))
        config = FollowingConfig(input_timeout_s=args.input_timeout,
                                 loss_timeout_s=args.loss_timeout)
    except ValueError as exc:
        parser.error(str(exc))

    try:
        import rclpy
        from rclpy.clock import Clock, ClockType
        from rclpy.node import Node
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from std_srvs.srv import SetBool
    except ImportError as exc:
        parser.exit(1, f"ROS 2 Jazzy environment required: {exc}\n"
                    "Run inside the Jetson container after sourcing /opt/ros/jazzy/setup.bash.\n")

    class CameraTest(Node):
        def __init__(self):
            super().__init__("person_following_camera_test")
            self.machine = FollowingStateMachine(controller, config)
            self.adapter = PerceptionAdapter(config.input_timeout_s)
            self.pending = None
            self.latest = None
            self.last_log = float("-inf")
            self.last_error_log = float("-inf")
            qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
            self.create_subscription(String, args.topic, self.on_tracks, qos)
            self.create_service(SetBool, "/person_following/set_enabled", self.on_enable)
            # The watchdog must keep running independently of sensor messages
            # and ROS time (which may be paused or adjusted).
            self.steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
            self.create_timer(1.0 / args.control_hz, self.tick, clock=self.steady_clock)
            self.get_logger().info(
                f"Camera-only test: subscribing to {args.topic}; NO velocity publishers. "
                "Keep the camera stationary and level. Enable service: /person_following/set_enabled")
            if args.enable_on_start:
                self.report(self.machine.enable(), time.monotonic())

        def on_enable(self, request, response):
            self.pending = None
            output = self.machine.enable() if request.data else self.machine.disable()
            self.report(output, time.monotonic())
            response.success = True
            response.message = f"{output.state.value}; camera test only, no drive output"
            return response

        def on_tracks(self, msg):
            source_now = self.get_clock().now().nanoseconds * 1e-9
            now = time.monotonic()
            try:
                frame = self.adapter.parse(msg.data, source_now_s=source_now,
                                           monotonic_now_s=now)
            except PerceptionError as exc:
                self.pending = None
                output = self.machine.update(now, input_healthy=False)
                self.report(output, now)
                if now - self.last_error_log >= 1.0:
                    self.get_logger().error(f"Rejected perception: {exc}")
                    self.last_error_log = now
                return
            if frame is not None:
                self.pending = frame
                self.latest = frame

        def tick(self):
            now = time.monotonic()
            frame, self.pending = self.pending, None
            self.report(self.machine.update(now, frame), now)

        def report(self, output, now):
            if output.event is None and now - self.last_log < 0.5:
                return
            self.last_log = now
            position = "-"
            age = "-"
            if self.latest is not None:
                age = f"{now - self.latest.timestamp_s:.3f}s"
                target = next((p for p in self.latest.people if p.track_id == output.target_id), None)
                if target is not None:
                    position = (f"({target.forward_m:.2f},{target.lateral_m:+.2f})m "
                                f"observed={target.observed}")
            self.get_logger().info(
                f"state={output.state.value} target={output.target_id} "
                f"v={output.command.linear_mps:.3f}m/s "
                f"w={output.command.angular_rps:+.3f}rad/s "
                f"position={position} frame_age={age} event={output.event or '-'}")

    rclpy.init(args=ros_args)
    node = CameraTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.report(node.machine.disable(), time.monotonic())
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
