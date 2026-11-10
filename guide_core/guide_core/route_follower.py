"""Route following: track progress along a Route and choose goals for Nav2.

A RouteFollower is created for one route. Each pose update projects the
robot onto the route, reports progress, and decides whether Nav2 should be
sent a new goal. Goals lie on the route a lookahead distance ahead, but are
held at sharp corners so the local planner never cuts across them.
"""

import math
from dataclasses import dataclass
from typing import Optional

from guide_core.route import Route

#: Goals whose distance along the route differs by less than this (m) are the same goal.
_SAME_S = 1e-9


@dataclass(frozen=True)
class FollowerConfig:
    """Tuning values for RouteFollower (all distances in metres).

    Attributes:
        lookahead: How far ahead of the robot, along the route, goals are placed.
        corner_angle: A waypoint turning by more than this (rad) is a corner.
        corner_release_distance: How close (along the route) the robot must get
            to a corner before the goal may move past it.
        goal_update_distance: Minimum movement of the goal along the route before
            a new goal is sent (corner and final goals are sent regardless).
        arrival_tolerance: The robot has arrived when both the distance remaining
            along the route and the straight-line distance to the end are within this.
        window_behind: The projection search window starts this far behind the last s.
        window_ahead: The projection search window ends this far ahead of the last s.

    Raises:
        ValueError: If a distance is not positive and finite, or corner_angle
            is outside [0, pi].
    """

    lookahead: float = 25.0
    corner_angle: float = math.radians(30.0)
    corner_release_distance: float = 1.5
    goal_update_distance: float = 5.0
    arrival_tolerance: float = 1.0
    window_behind: float = 5.0
    window_ahead: float = 30.0

    def __post_init__(self):
        for name in ('lookahead', 'corner_release_distance', 'goal_update_distance',
                     'arrival_tolerance', 'window_behind', 'window_ahead'):
            value = getattr(self, name)
            if not (math.isfinite(value) and value > 0.0):
                raise ValueError(f'{name} must be a positive finite number, got {value}')
        if not 0.0 <= self.corner_angle <= math.pi:  # also rejects NaN
            raise ValueError(f'corner_angle must be in [0, pi], got {self.corner_angle}')


@dataclass(frozen=True)
class Goal:
    """A goal pose on the route, in the map frame.

    Attributes:
        x: East (m).
        y: North (m).
        yaw: Heading of the route at the goal (rad, REP 103); at a corner, the
            outgoing direction.
        s: Distance along the route of the goal (m).
        is_corner: True if the goal is being held at a corner.
        is_final: True if the goal is the end of the route.
    """

    x: float
    y: float
    yaw: float
    s: float
    is_corner: bool
    is_final: bool


@dataclass(frozen=True)
class FollowerOutput:
    """Result of one RouteFollower.update() call.

    The progress fields match guide_interfaces/msg/RouteProgress.

    Attributes:
        s: Distance along the route of the robot's projected position (m).
        segment_index: Segment the robot is projected onto.
        cross_track_error: Signed distance from the route (m), + = left.
        distance_remaining: Distance along the route still to go (m).
        total_length: Total route length (m).
        arrived: True once the robot has reached the end (latched).
        new_goal: A goal to send to Nav2 now, or None if the current one stands.
    """

    s: float
    segment_index: int
    cross_track_error: float
    distance_remaining: float
    total_length: float
    arrived: bool
    new_goal: Optional[Goal]


class RouteFollower:
    """Follows one route: reports progress and chooses goals.

    Create a new RouteFollower for each new route; there is no reset.

    Args:
        route: The route to follow.
        config: Tuning values; defaults to FollowerConfig().
    """

    def __init__(self, route: Route, config: FollowerConfig = FollowerConfig()):
        self._route = route
        self._config = config
        # Distance along the route of every corner, in order.
        self._corner_s = tuple(
            route.cumulative[i] for i in route.corners(config.corner_angle))
        self._last_s: Optional[float] = None   # None until the first update
        self._released_s = -math.inf           # corners at or before this s are released
        self._goal: Optional[Goal] = None      # last goal handed out
        self._arrived = False

    @property
    def route(self) -> Route:
        """The route being followed."""
        return self._route

    @property
    def config(self) -> FollowerConfig:
        """The tuning values in use."""
        return self._config

    @property
    def goal(self) -> Optional[Goal]:
        """The most recent goal handed out, or None before the first one."""
        return self._goal

    @property
    def arrived(self) -> bool:
        """True once the robot has reached the end of the route."""
        return self._arrived

    def update(self, x: float, y: float) -> FollowerOutput:
        """Process one robot position (map frame, metres).

        Raises:
            ValueError: If x or y is not finite.
        """
        if not (math.isfinite(x) and math.isfinite(y)):
            raise ValueError(f'robot position must be finite, got ({x}, {y})')
        cfg = self._config
        route = self._route

        # 1. Project onto the route; after the first update, only search a
        #    window around the last known progress (see Route.project).
        if self._last_s is None:
            projection = route.project(x, y)
        else:
            projection = route.project(
                x, y,
                s_min=self._last_s - cfg.window_behind,
                s_max=self._last_s + cfg.window_ahead)
        s = projection.s
        self._last_s = s
        remaining = route.total_length - s

        # 2. Arrival needs both conditions, so a route that loops back to its
        #    start is not "arrived" at the beginning. Latched once true.
        if not self._arrived:
            near_end = math.dist((x, y), route.points[-1]) <= cfg.arrival_tolerance
            if remaining <= cfg.arrival_tolerance and near_end:
                self._arrived = True

        # 3-5. Choose a goal and decide whether to hand it out.
        new_goal = None
        if not self._arrived:
            self._release_corners(s)
            candidate = self._candidate_goal(s)
            if self._should_send(candidate):
                self._goal = candidate
                new_goal = candidate

        return FollowerOutput(
            s=s,
            segment_index=projection.segment_index,
            cross_track_error=projection.cross_track_error,
            distance_remaining=remaining,
            total_length=route.total_length,
            arrived=self._arrived,
            new_goal=new_goal,
        )

    def _release_corners(self, s: float) -> None:
        """Release every corner the robot is within corner_release_distance of.

        Why the release distance is 1.5 m (see the Step 4 design notes):
        once a corner is released, Nav2 aims at a goal ~lookahead metres down
        the next leg and plans roughly straight to it, cutting inside the
        corner by about corner_release_distance (r * L / sqrt(r^2 + L^2) ~= r
        for a 90 degree turn). So it must be small, or the robot leaves the
        pavement. But it must stay comfortably above Nav2's goal tolerance
        (~0.25 m; the sim stops 0.3 m short), otherwise the robot parks just
        before the corner and it is never released (deadlock), and the
        controller slows to a near-stop at every corner. GPS noise (Step 8)
        also needs margin, so revisit this value then. Distances are measured
        along the route so sideways drift does not affect the release.
        Step 5 adds a safety net: Nav2 reporting SUCCEEDED for a corner goal
        also releases it. If corner cutting is still a problem on the real
        robot, Nav2's NavigateThroughPoses action is the alternative (Step 9).

        Releases are latched (they never un-release), so a pose wobbling
        across the threshold cannot make the goal flip back and forth.
        """
        for corner_s in self._corner_s:
            if corner_s > self._released_s and \
                    s >= corner_s - self._config.corner_release_distance:
                self._released_s = corner_s

    def _candidate_goal(self, s: float) -> Goal:
        """Return the goal for progress s: lookahead ahead, held at the next corner."""
        route = self._route
        total = route.total_length
        target_s = min(s + self._config.lookahead, total)

        # The first corner not yet released blocks the goal from passing it.
        blocking = next((c for c in self._corner_s if c > self._released_s), None)
        is_corner = blocking is not None and target_s >= blocking
        if is_corner:
            target_s = blocking

        x, y = route.point_at(target_s)
        return Goal(
            x=x,
            y=y,
            yaw=route.heading_at(target_s),
            s=target_s,
            is_corner=is_corner,
            is_final=target_s >= total - _SAME_S,
        )

    def _should_send(self, candidate: Goal) -> bool:
        """Decide whether candidate replaces the current goal."""
        if self._goal is None:
            return True
        change = abs(candidate.s - self._goal.s)
        if change < _SAME_S:
            return False  # same goal: never resend
        if candidate.is_corner or candidate.is_final:
            return True   # Nav2 must aim exactly at corners and the end
        return change >= self._config.goal_update_distance