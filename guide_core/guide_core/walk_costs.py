"""Walkway and crossing classification, and edge costs for route planning.

Works on plain dictionaries of OpenStreetMap tags, as stored on the nodes and
edges of an osmnx graph, so it needs neither osmnx nor networkx.

Cost model (all values in metres):

    cost = length * road factor + crossing penalty

- Steps and indoor corridors are forbidden: the robot cannot use them.
- Walking along a road centreline is discouraged: minor roads x3, major x10.
- A crossing is charged once, on the walkway edge that steps onto the
  crossing node, with a penalty that depends on the crossing type and on the
  class of road crossed (node attribute 'crossed_road', added by
  fetch_osm_graph; missing means unknown and is treated as major).
"""

import enum
import math
from dataclasses import dataclass, fields

FORBIDDEN_TYPES = frozenset({'steps', 'corridor'})
WALKWAY_TYPES = frozenset({'footway', 'pedestrian', 'path'})
MAJOR_ROADS = frozenset({
    'trunk', 'trunk_link', 'primary', 'primary_link',
    'secondary', 'secondary_link', 'tertiary', 'tertiary_link',
})
MINOR_ROADS = frozenset({'residential', 'unclassified', 'service', 'living_street', 'road'})

# UK crossing_ref values for crossings with traffic lights.
SIGNALLED_CROSSING_REFS = frozenset({'pelican', 'puffin', 'toucan', 'pegasus'})
# crossing=* values that mean "no traffic lights".
UNSIGNALLED_CROSSING_VALUES = frozenset({'unmarked', 'uncontrolled', 'marked', 'zebra', 'informal'})


class WalkwayClass(enum.Enum):
    """How an edge may be used, from its highway=* tag."""

    FORBIDDEN = 'forbidden'
    WALKWAY = 'walkway'
    MINOR_ROAD = 'minor_road'
    MAJOR_ROAD = 'major_road'


class CrossingType(enum.Enum):
    """What controls a crossing."""

    SIGNALLED = 'signalled'
    UNSIGNALLED = 'unsignalled'
    UNKNOWN = 'unknown'


def _norm(value):
    """Return value as a stripped lower-case string, or None if missing/empty."""
    if value is None:
        return None
    text = str(value).strip().lower()
    return text or None


def _tag(tags, key):
    """Return the normalised value of tags[key], or None (tags may be None)."""
    if not tags:
        return None
    return _norm(tags.get(key))


def walkway_class(highway):
    """Classify a highway=* value.

    Unrecognised or missing values (e.g. bridleway) count as minor roads:
    usable, but discouraged.
    """
    value = _norm(highway)
    if value in FORBIDDEN_TYPES:
        return WalkwayClass.FORBIDDEN
    if value in WALKWAY_TYPES:
        return WalkwayClass.WALKWAY
    if value in MAJOR_ROADS:
        return WalkwayClass.MAJOR_ROAD
    return WalkwayClass.MINOR_ROAD


def is_road(highway):
    """Return True if a highway=* value is a road that cars use."""
    value = _norm(highway)
    return value in MAJOR_ROADS or value in MINOR_ROADS


def classify_crossing(*tag_dicts):
    """Classify a crossing from one or more tag dictionaries (node, way).

    Evidence from all dictionaries is combined. If the tags contradict each
    other, the result is UNSIGNALLED: the safer assumption for a blind user.
    None or empty dictionaries are ignored.
    """
    signalled = False
    unsignalled = False
    for tags in tag_dicts:
        crossing = _tag(tags, 'crossing')
        signals = _tag(tags, 'crossing:signals')
        ref = _tag(tags, 'crossing_ref')
        if crossing == 'traffic_signals' or signals == 'yes' or ref in SIGNALLED_CROSSING_REFS:
            signalled = True
        if crossing in UNSIGNALLED_CROSSING_VALUES or signals == 'no' or ref == 'zebra':
            unsignalled = True
    if unsignalled:
        return CrossingType.UNSIGNALLED
    if signalled:
        return CrossingType.SIGNALLED
    return CrossingType.UNKNOWN


@dataclass(frozen=True)
class CostConfig:
    """Tunable weights of the cost model, in metres of extra walking.

    Road factors must be >= 1 so a road is never cheaper than a walkway of
    the same length (and the straight-line distance stays a lower bound on
    cost). Penalties must be finite and >= 0, as Dijkstra requires.
    """

    minor_road_factor: float = 3.0
    major_road_factor: float = 10.0
    signalled_minor: float = 10.0
    unsignalled_minor: float = 30.0
    unknown_minor: float = 40.0
    signalled_major: float = 20.0
    unsignalled_major: float = 300.0
    unknown_major: float = 300.0

    def __post_init__(self):
        """Validate all values."""
        for f in fields(self):
            value = getattr(self, f.name)
            if isinstance(value, bool) or not isinstance(value, (int, float)) \
                    or not math.isfinite(value):
                raise ValueError(f'{f.name} must be a finite number, got {value!r}')
            minimum = 1.0 if f.name.endswith('_factor') else 0.0
            if value < minimum:
                raise ValueError(f'{f.name} must be >= {minimum}, got {value!r}')


def crossing_penalty(crossing_type, crossed_road, config):
    """Return the penalty for one crossing.

    crossed_road: 'major' or 'minor' (class of road crossed), or 'none' (we
    looked and no public road passes through it, e.g. a cycle track): 'none'
    uses the minor penalties. Missing or anything else (no information)
    counts as major, the cautious assumption.
    """
    minor = _norm(crossed_road) in ('minor', 'none')
    if crossing_type is CrossingType.SIGNALLED:
        return config.signalled_minor if minor else config.signalled_major
    if crossing_type is CrossingType.UNSIGNALLED:
        return config.unsignalled_minor if minor else config.unsignalled_major
    return config.unknown_minor if minor else config.unknown_major


def edge_cost(length, edge_tags, from_tags, to_tags, config=None):
    """Return the cost of walking one directed edge, or None if forbidden.

    length: edge length in metres. edge_tags: the edge's (way's) tags.
    from_tags / to_tags: tags of the start / end node (may be None).
    """
    if config is None:
        config = CostConfig()
    length = float(length)
    if not (math.isfinite(length) and length >= 0.0):
        raise ValueError(f'length must be finite and >= 0, got {length!r}')

    highway = _tag(edge_tags, 'highway')
    walk_class = walkway_class(highway)
    if walk_class is WalkwayClass.FORBIDDEN:
        return None
    factor = {
        WalkwayClass.WALKWAY: 1.0,
        WalkwayClass.MINOR_ROAD: config.minor_road_factor,
        WalkwayClass.MAJOR_ROAD: config.major_road_factor,
    }[walk_class]
    cost = length * factor

    # Walking along a road through a crossing point is not crossing that road.
    if is_road(highway):
        return cost
    # Stepping onto a crossing node: charge the crossing once, here.
    if _tag(to_tags, 'highway') == 'crossing':
        crossing_type = classify_crossing(to_tags, edge_tags)
        return cost + crossing_penalty(crossing_type, _tag(to_tags, 'crossed_road'), config)
    # Fallback: a crossing way with no crossing node at either end.
    if _tag(edge_tags, 'footway') == 'crossing' and _tag(from_tags, 'highway') != 'crossing':
        return cost + crossing_penalty(classify_crossing(edge_tags), None, config)
    return cost