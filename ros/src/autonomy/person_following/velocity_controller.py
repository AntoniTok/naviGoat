"""Hardware-independent stage-one following controller.

Positions are metres from the camera: forward is positive ahead, lateral is
positive left. Angular velocity is positive for a left turn. This module does
not publish commands or handle target loss; its caller must supply fresh,
observed positions and enforce stopping when observations are unavailable.
"""

from __future__ import annotations

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class VelocityCommand:
    linear_mps: float = 0.0
    angular_rps: float = 0.0


@dataclass(frozen=True)
class ControllerConfig:
    following_distance_m: float = 1.0
    distance_tolerance_m: float = 0.15
    max_linear_mps: float = 0.2
    max_angular_rps: float = 0.5
    distance_gain: float = 0.5
    bearing_gain: float = 1.0
    turn_in_place_angle_rad: float = math.radians(30.0)

    def __post_init__(self):
        for name, value in vars(self).items():
            if not math.isfinite(value) or value <= 0:
                raise ValueError(f"{name} must be finite and positive")
        if self.distance_tolerance_m >= self.following_distance_m:
            raise ValueError("Distance tolerance must be smaller than following distance")
        if self.turn_in_place_angle_rad >= math.pi / 2:
            raise ValueError("Turn-in-place angle must be smaller than pi/2")


class VelocityController:
    """Proportional distance/bearing control, without reverse motion.

    Forward distance uses the same camera-forward convention as Enid's tracker.
    Steering remains active inside the distance band. For larger bearings the
    robot turns in place rather than driving towards a strongly offset target.
    """

    def __init__(self, config: ControllerConfig | None = None):
        self.config = config if config is not None else ControllerConfig()

    def compute(self, forward_m: float, lateral_m: float) -> VelocityCommand:
        """Return a bounded command, or zero for invalid/behind-camera positions."""
        try:
            forward = float(forward_m)
            lateral = float(lateral_m)
        except (TypeError, ValueError, OverflowError):
            return VelocityCommand()
        if not math.isfinite(forward) or not math.isfinite(lateral) or forward <= 0:
            return VelocityCommand()

        cfg = self.config
        bearing = math.atan2(lateral, forward)
        angular = max(-cfg.max_angular_rps,
                      min(cfg.max_angular_rps, cfg.bearing_gain * bearing))

        # Subtract the tolerance so speed rises continuously at the band edge.
        excess_distance = max(
            0.0, forward - cfg.following_distance_m - cfg.distance_tolerance_m
        )
        linear = min(cfg.max_linear_mps, cfg.distance_gain * excess_distance)
        if abs(bearing) >= cfg.turn_in_place_angle_rad:
            linear = 0.0
        else:
            linear *= math.cos(bearing)
        return VelocityCommand(linear, angular)
