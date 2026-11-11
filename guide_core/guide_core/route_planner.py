"""Route planning on an OpenStreetMap walking graph.

Works on a networkx MultiDiGraph shaped like an osmnx graph: node attributes
x (longitude), y (latitude) and OSM tags; edge attributes = the way's OSM tags.
Needs neither osmnx nor ROS.

The graph must be unsimplified: every edge is a straight line between its two
nodes, so an edge's length is the straight-line distance in the local
east-north (ENU) frame around the origin.
"""

import math
from dataclasses import dataclass, field

import networkx as nx

from guide_core.geo import latlon_to_enu
from guide_core.polyline import simplify_polyline
from guide_core.route import MIN_SEGMENT_LENGTH, Route
from guide_core.walk_costs import CostConfig, classify_crossing, edge_cost, is_road


class PlanningError(ValueError):
    """No route can be planned. The message says why, in plain words."""


def _value(tags, key):
    """Return tags[key] as a stripped lower-case string, or None."""
    value = tags.get(key) if tags else None
    if value is None:
        return None
    text = str(value).strip().lower()
    return text or None


def _is_finite_number(value):
    """Return True for a finite int/float that is not a bool."""
    return (not isinstance(value, bool) and isinstance(value, (int, float))
            and math.isfinite(value))


@dataclass(frozen=True)
class PlannerConfig:
    """Planner settings.

    max_snap_distance: furthest (m) a start/goal may be from a usable graph
    node. simplify_tolerance: Douglas-Peucker tolerance (m).
    crossing_half_width: assumed half-width (m) of a crossing known only from
    its crossing node (no mapped crossing way). costs: edge cost weights.
    """

    max_snap_distance: float = 30.0
    simplify_tolerance: float = 0.5
    crossing_half_width: float = 3.0
    costs: CostConfig = field(default_factory=CostConfig)

    def __post_init__(self):
        """Validate all values."""
        if not _is_finite_number(self.max_snap_distance) or self.max_snap_distance <= 0.0:
            raise ValueError(f'max_snap_distance must be finite and > 0, '
                             f'got {self.max_snap_distance!r}')
        if not _is_finite_number(self.simplify_tolerance) or self.simplify_tolerance < 0.0:
            raise ValueError(f'simplify_tolerance must be finite and >= 0, '
                             f'got {self.simplify_tolerance!r}')
        if not _is_finite_number(self.crossing_half_width) or self.crossing_half_width <= 0.0:
            raise ValueError(f'crossing_half_width must be finite and > 0, '
                             f'got {self.crossing_half_width!r}')
        if not isinstance(self.costs, CostConfig):
            raise ValueError(f'costs must be a CostConfig, got {self.costs!r}')


@dataclass(frozen=True)
class RouteCrossing:
    """One road crossing along a planned route.

    start_s / end_s: distance (m) along the route of the near and far kerb.
    crossed_road: 'major', 'minor', 'none' (no public road, e.g. a cycle
    track) or 'unknown'.
    """

    start_s: float
    end_s: float
    crossing_type: object  # guide_core.walk_costs.CrossingType
    crossed_road: str
    tactile_paving: bool


@dataclass(frozen=True)
class PlannedRoute:
    """The result of a successful plan.

    route: the simplified route in the map (ENU) frame.
    latlons: (lat, lon) of each route point, in the same order.
    crossings: RouteCrossing tuple, in route order.
    """

    route: Route
    latlons: tuple
    start_snap_distance: float
    goal_snap_distance: float
    crossings: tuple

    @property
    def length(self):
        """Route length in metres."""
        return self.route.total_length


class RoutePlanner:
    """Plans walking routes on one graph. Create once; call plan() many times."""

    def __init__(self, graph, origin_lat, origin_lon, config=None):
        """Precompute node positions, edge costs and usable nodes.

        Raises ValueError if a node lacks numeric coordinates or nothing is usable.
        """
        self._config = PlannerConfig() if config is None else config
        if not isinstance(self._config, PlannerConfig):
            raise ValueError(f'config must be a PlannerConfig, got {config!r}')
        self._graph = graph
        self._origin = (float(origin_lat), float(origin_lon))

        # Every node's position in metres east/north of the origin.
        self._xy = {}
        for node, data in graph.nodes(data=True):
            try:
                lon, lat = float(data['x']), float(data['y'])
            except (KeyError, TypeError, ValueError):
                raise ValueError(f'graph node {node!r} has no numeric x (lon) / y (lat)') from None
            if not (math.isfinite(lat) and math.isfinite(lon)):
                raise ValueError(f'graph node {node!r} has non-finite coordinates')
            self._xy[node] = latlon_to_enu(lat, lon, *self._origin)

        # Cost of each directed node pair; for parallel edges keep the cheapest
        # usable one (and its tags). Forbidden edges get no entry at all.
        self._cost = {}
        self._edge_tags = {}
        for u, v, data in graph.edges(data=True):
            length = math.dist(self._xy[u], self._xy[v])
            cost = edge_cost(length, data, graph.nodes[u], graph.nodes[v], self._config.costs)
            if cost is None:
                continue
            if (u, v) not in self._cost or cost < self._cost[(u, v)]:
                self._cost[(u, v)] = cost
                self._edge_tags[(u, v)] = data

        # Only nodes touching at least one usable edge can be snapped to.
        usable = {u for u, _ in self._cost} | {v for _, v in self._cost}
        self._usable = [(node, self._xy[node]) for node in graph.nodes if node in usable]
        if not self._usable:
            raise ValueError('graph has no usable walkways')

    @property
    def config(self):
        """The PlannerConfig in use."""
        return self._config

    @property
    def usable_node_count(self):
        """Number of nodes a start or goal can snap to."""
        return len(self._usable)

    def plan(self, start_lat, start_lon, goal_lat, goal_lon):
        """Plan a route between two lat/lon points. Returns a PlannedRoute.

        Raises PlanningError (with a readable message) if no route is possible.
        """
        for value in (start_lat, start_lon, goal_lat, goal_lon):
            if not _is_finite_number(value):
                raise PlanningError(f'coordinates must be finite numbers, got {value!r}')
        start_node, start_dist = self._snap(start_lat, start_lon, 'start')
        goal_node, goal_dist = self._snap(goal_lat, goal_lon, 'goal')
        if start_node == goal_node:
            raise PlanningError('Start and goal are at the same point on the walking graph '
                                '(already at the destination).')

        try:
            path = nx.dijkstra_path(self._graph, start_node, goal_node, weight=self._weight)
        except nx.NetworkXNoPath:
            raise PlanningError('No walkable route between start and goal '
                                '(steps and indoor corridors are not allowed).') from None

        crossing_spans = self._find_crossings(path)

        # Simplify, always keeping the kerb points of mapped crossing ways, and
        # the node itself for crossings known only from a crossing node.
        xy = [self._xy[node] for node in path]
        keep = set()
        for first, last, centre, *_ in crossing_spans:
            keep.update((first, last) if centre is None else (centre,))
        kept = simplify_polyline(xy, self._config.simplify_tolerance, keep=keep)

        # Build the point and lat/lon lists, merging practically identical
        # points exactly as Route would, so both lists stay the same length.
        # position[path index] = index of the matching route point.
        points, latlons, position = [], [], {}
        for i in kept:
            if points and math.dist(points[-1], xy[i]) < MIN_SEGMENT_LENGTH:
                position[i] = len(points) - 1
                continue
            data = self._graph.nodes[path[i]]
            points.append(xy[i])
            latlons.append((float(data['y']), float(data['x'])))
            position[i] = len(points) - 1
        try:
            route = Route(points)
        except ValueError as e:
            raise PlanningError(f'Planned path is not a valid route: {e}') from e

        # Distance along the route of every route point.
        along = [0.0]
        for a, b in zip(route.points, route.points[1:]):
            along.append(along[-1] + math.dist(a, b))
        total = along[-1]

        half = self._config.crossing_half_width
        crossings = []
        for first, last, centre, crossing_type, crossed_road, tactile in crossing_spans:
            if centre is None:  # mapped crossing way: kerb to kerb
                start_s, end_s = along[position[first]], along[position[last]]
            else:  # crossing node only: assume half-width either side of it
                s_node = along[position[centre]]
                start_s, end_s = max(0.0, s_node - half), min(total, s_node + half)
            crossings.append(RouteCrossing(start_s=start_s, end_s=end_s,
                                           crossing_type=crossing_type,
                                           crossed_road=crossed_road, tactile_paving=tactile))

        return PlannedRoute(route=route, latlons=tuple(latlons),
                            start_snap_distance=start_dist, goal_snap_distance=goal_dist,
                            crossings=tuple(crossings))

    def _weight(self, u, v, _data):
        """Dijkstra edge weight; None hides the edge (forbidden)."""
        return self._cost.get((u, v))

    def _snap(self, lat, lon, what):
        """Return (nearest usable node, distance in m), or raise PlanningError if too far."""
        x, y = latlon_to_enu(lat, lon, *self._origin)
        best_node, best_dist = None, math.inf
        for node, (node_x, node_y) in self._usable:
            d = math.hypot(node_x - x, node_y - y)
            if d < best_dist:
                best_node, best_dist = node, d
        limit = self._config.max_snap_distance
        if best_dist > limit:
            raise PlanningError(f'The {what} is {best_dist:.1f} m from the nearest walkway '
                                f'(limit {limit:.1f} m).')
        return best_node, best_dist

    def _find_crossings(self, path):
        """Return (first, last, centre, type, crossed_road, tactile) per crossing on the path.

        A crossing is either a run of consecutive footway=crossing edges
        (first/last = path indices of the near and far kerb, centre = None),
        or a crossing node reached from a walkway outside any such run
        (centre = its path index; first/last = its neighbours, used only to
        collect tags).
        """
        graph = self._graph
        n_edges = len(path) - 1
        is_crossing_edge = [
            _value(self._edge_tags[(path[i], path[i + 1])], 'footway') == 'crossing'
            for i in range(n_edges)
        ]

        spans = []
        covered = set()
        i = 0
        while i < n_edges:
            if is_crossing_edge[i]:
                j = i
                while j + 1 < n_edges and is_crossing_edge[j + 1]:
                    j += 1
                spans.append((i, j + 1, None))  # edges i..j join path nodes i..j+1
                covered.update(range(i, j + 2))
                i = j + 1
            else:
                i += 1
        for k in range(1, len(path)):
            if k in covered or _value(graph.nodes[path[k]], 'highway') != 'crossing':
                continue
            if is_road(self._edge_tags[(path[k - 1], path[k])].get('highway')):
                continue  # walking along a road through a crossing point
            spans.append((k - 1, min(k + 1, len(path) - 1), k))
        spans.sort(key=lambda span: span[0])

        result = []
        for first, last, centre in spans:
            node_tags = [graph.nodes[path[k]] for k in range(first, last + 1)
                         if _value(graph.nodes[path[k]], 'highway') == 'crossing']
            edge_tags = [self._edge_tags[(path[k], path[k + 1])] for k in range(first, last)]
            crossing_type = classify_crossing(*node_tags, *edge_tags)
            roads = {_value(tags, 'crossed_road') for tags in node_tags}
            crossed_road = ('major' if 'major' in roads else 'minor' if 'minor' in roads
                            else 'none' if 'none' in roads else 'unknown')
            tactile = any(_value(tags, 'tactile_paving') == 'yes' for tags in node_tags + edge_tags)
            result.append((first, last, centre, crossing_type, crossed_road, tactile))
        return result