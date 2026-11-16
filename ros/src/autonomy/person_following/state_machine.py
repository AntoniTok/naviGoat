"""Plain-Python following lifecycle; no ROS, camera or wall-clock dependencies.

Call enable() explicitly, then update() regularly even when no frames arrive.
Pass a PerceptionFrame for each NEW perception result; an empty people tuple
means a working camera saw nobody. frame=None means no new result has arrived.
All timestamps must use the same monotonic clock, in seconds. A future ROS
adapter must convert sensor timestamps into this clock domain.

Predicted tracks do not count as sightings. ID continuity is trusted only for
the short loss window; callers must provide reliable association. This module
cannot establish a person's identity from their position or track ID alone.
"""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
import math

from velocity_controller import VelocityCommand, VelocityController


class FollowingState(str, Enum):
    IDLE = "IDLE"
    ACQUIRING = "ACQUIRING"
    FOLLOWING = "FOLLOWING"
    TEMPORARILY_LOST = "TEMPORARILY_LOST"


@dataclass(frozen=True)
class PersonObservation:
    track_id: int
    forward_m: float
    lateral_m: float
    observed: bool = True
    confirmed: bool = True


@dataclass(frozen=True)
class PerceptionFrame:
    timestamp_s: float
    people: tuple[PersonObservation, ...] = ()


@dataclass(frozen=True)
class FollowingConfig:
    loss_ramp_s: float = 0.5
    loss_timeout_s: float = 2.0
    input_timeout_s: float = 0.5
    linear_acceleration_mps2: float = 0.4
    angular_acceleration_rps2: float = 1.0

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.loss_ramp_s >= self.loss_timeout_s:
            raise ValueError("Loss ramp must finish before the loss timeout")


@dataclass(frozen=True)
class FollowingOutput:
    state: FollowingState
    command: VelocityCommand
    target_id: int | None
    # One-shot machine-readable event, for a future adapter to publish to Lia.
    event: str | None = None


class FollowingStateMachine:
    def __init__(self, controller: VelocityController | None = None,
                 config: FollowingConfig | None = None):
        self.controller = controller if controller is not None else VelocityController()
        self.config = config if config is not None else FollowingConfig()
        self._reset()
        self._last_tick = None

    def _reset(self):
        self.state = FollowingState.IDLE
        self.target_id = None
        self.command = VelocityCommand()
        self._last_frame = None
        self._last_seen = None
        self._target = None
        self._loss_started = None
        self._loss_command = VelocityCommand()

    def _output(self, event=None):
        return FollowingOutput(self.state, self.command, self.target_id, event)

    def _stop(self, reason):
        self._reset()
        return self._output(reason)

    def enable(self) -> FollowingOutput:
        """Start acquisition; repeated enable calls do not reset an active target."""
        if self.state != FollowingState.IDLE:
            return self._output()
        self._reset()
        self.state = FollowingState.ACQUIRING
        self._last_tick = None
        return self._output("enabled")

    def disable(self) -> FollowingOutput:
        """Immediately command zero (also suitable for an external stop request)."""
        return self._stop("disabled")

    def update(self, now_s: float, frame: PerceptionFrame | None = None,
               *, input_healthy: bool = True) -> FollowingOutput:
        """Advance one control tick and return a command plus optional event.

        Camera/depth failures should set input_healthy=False. Stale input also
        immediately stops and exits. Ordinary missed detections instead use the
        loss ramp. The caller remains responsible for transmitting each output.
        """
        if (not math.isfinite(now_s) or
                (self._last_tick is not None and now_s < self._last_tick)):
            self._stop("input_fault")
            raise ValueError("Control time must be finite and monotonic")
        dt = 0.0 if self._last_tick is None else now_s - self._last_tick
        self._last_tick = now_s
        if self.state == FollowingState.IDLE:
            return self._output()
        if not input_healthy:
            return self._stop("input_fault")

        # Enforce the deadline BEFORE accepting a returning target. A late
        # reappearance must not silently reactivate a completed following mode.
        if (self._last_seen is not None and
                now_s - self._last_seen >= self.config.loss_timeout_s):
            return self._stop("target_lost")

        event = None
        if frame is not None:
            if not math.isfinite(frame.timestamp_s) or frame.timestamp_s > now_s:
                return self._stop("input_fault")
            # Repeated/out-of-order frames cannot refresh the watchdog.
            is_new = self._last_frame is None or frame.timestamp_s > self._last_frame
            if is_new:
                if now_s - frame.timestamp_s > self.config.input_timeout_s:
                    return self._stop("input_stale")
                self._last_frame = frame.timestamp_s
                observed = [p for p in frame.people if p.observed]
                ids = [p.track_id for p in observed]
                if (any(not isinstance(i, int) or isinstance(i, bool) or i < 1 for i in ids)
                        or len(set(ids)) != len(ids)):
                    return self._stop("input_fault")
                for person in observed:
                    if (not math.isfinite(person.forward_m) or
                            not math.isfinite(person.lateral_m) or person.forward_m <= 0):
                        return self._stop("input_fault")
                candidates = [p for p in observed if p.confirmed]
                target = next((p for p in candidates if p.track_id == self.target_id), None)
                if self.state == FollowingState.ACQUIRING and candidates:
                    # Choose once at acquisition, then use only the selected ID.
                    # Horizontal camera range includes sideways displacement;
                    # use ID as a deterministic tie-break for equal distances.
                    target = min(candidates, key=lambda p: (
                        math.hypot(p.forward_m, p.lateral_m), p.track_id))
                    self.target_id = target.track_id
                    event = "target_acquired"
                if target is not None:
                    if self.state == FollowingState.TEMPORARILY_LOST:
                        event = "target_reacquired"
                    self.state = FollowingState.FOLLOWING
                    self._last_seen = frame.timestamp_s
                    self._target = target
                    self._loss_started = None
                elif self.state == FollowingState.FOLLOWING:
                    self.state = FollowingState.TEMPORARILY_LOST
                    self._loss_started = now_s
                    self._loss_command = self.command
                    event = "target_missing"

        if (self._last_frame is not None and
                now_s - self._last_frame > self.config.input_timeout_s):
            return self._stop("input_stale")

        if self.state == FollowingState.FOLLOWING:
            desired = self.controller.compute(self._target.forward_m, self._target.lateral_m)
            self.command = self._smooth_command(desired, dt)
        elif self.state == FollowingState.TEMPORARILY_LOST:
            # Always scale the command captured at loss onset, rather than
            # repeatedly scaling a changing value (which would depend on rate).
            fraction = max(0.0, 1.0 - (now_s - self._loss_started) / self.config.loss_ramp_s)
            self.command = VelocityCommand(
                self._loss_command.linear_mps * fraction,
                self._loss_command.angular_rps * fraction,
            )
        else:
            self.command = VelocityCommand()
        return self._output(event)

    def _smooth_command(self, desired, dt):
        def approach(current, requested, acceleration):
            # A zero forward request (too close / turn in place) stops promptly.
            if requested == 0:
                return 0.0
            limit = acceleration * dt
            return current + max(-limit, min(limit, requested - current))

        return VelocityCommand(
            approach(self.command.linear_mps, desired.linear_mps,
                     self.config.linear_acceleration_mps2),
            approach(self.command.angular_rps, desired.angular_rps,
                     self.config.angular_acceleration_rps2),
        )
