"""Tests for crossing a major road by walking along a minor road through a junction.

walk_costs.edge_cost charges it as an unknown crossing of a major road, and
route_planner reports it as a crossing (so the robot stops before it).
"""

import networkx as nx
import pytest

from guide_core.geo import enu_to_latlon
from guide_core.route_planner import PlannerConfig, RoutePlanner
from guide_core.walk_costs import CrossingType, edge_cost, is_major_junction

ORIGIN = (51.4988, -0.1749)
JUNCTION = {'junction_major': 'yes'}
MINOR_ROAD = {'highway': 'residential'}
FOOTWAY = {'highway': 'footway'}


# --- walk_costs -------------------------------------------------------------------

def test_minor_road_into_major_junction_pays_unknown_major():
    assert edge_cost(5.0, MINOR_ROAD, {}, JUNCTION) == pytest.approx(5 * 10 + 300)


def test_major_road_into_junction_pays_only_its_factor():
    assert edge_cost(5.0, {'highway': 'primary'}, {}, JUNCTION) == pytest.approx(5 * 50)


def test_footway_into_junction_node_not_charged():
    assert edge_cost(5.0, FOOTWAY, {}, JUNCTION) == pytest.approx(5.0)


def test_minor_road_into_ordinary_node_unchanged():
    assert edge_cost(5.0, MINOR_ROAD, {}, {}) == pytest.approx(50.0)


@pytest.mark.parametrize('tags, expected', [
    ({'junction_major': 'yes'}, True),
    ({'junction_major': 'no'}, False),
    ({}, False),
])
def test_is_major_junction(tags, expected):
    assert is_major_junction(tags) is expected


# --- route_planner ----------------------------------------------------------------

def _graph(detour):
    """A minor road N-J-S (30 m) straight across a major road at junction J (0, 0).

    With detour set, there is also a pavement route via a signalled crossing
    `detour` m east: N -> NE -> crossing -> SE -> S, cost 2 x detour + 30 + 20.
    Straight down the road: 30 x 10 + 300 = 600.
    """
    g = nx.MultiDiGraph()

    def node(name, east, north, **tags):
        lat, lon = enu_to_latlon(east, north, *ORIGIN)
        g.add_node(name, x=lon, y=lat, **tags)

    def way(a, b, tags):
        g.add_edge(a, b, **tags)
        g.add_edge(b, a, **tags)

    node('N', 0, 15)
    node('J', 0, 0, **JUNCTION)
    node('S', 0, -15)
    way('N', 'J', MINOR_ROAD)
    way('J', 'S', MINOR_ROAD)
    if detour is not None:
        node('NE', detour, 15)
        node('C', detour, 0, highway='crossing', crossing='traffic_signals',
             crossed_road='major')
        node('SE', detour, -15)
        way('N', 'NE', FOOTWAY)
        way('NE', 'C', {'highway': 'footway', 'footway': 'crossing'})
        way('C', 'SE', {'highway': 'footway', 'footway': 'crossing'})
        way('SE', 'S', FOOTWAY)
    return g


def _plan(graph, config=None):
    planner = RoutePlanner(graph, *ORIGIN, config)
    return planner.plan(*enu_to_latlon(0, 15, *ORIGIN), *enu_to_latlon(0, -15, *ORIGIN))


def test_planner_prefers_signalled_crossing_over_junction():
    planned = _plan(_graph(detour=100.0))                # 250 < 600
    assert max(x for x, _ in planned.route.points) == pytest.approx(100.0, abs=0.01)
    assert [c.crossing_type for c in planned.crossings] == [CrossingType.SIGNALLED]


def test_junction_reported_as_crossing_when_only_way():
    planned = _plan(_graph(detour=None))
    assert len(planned.crossings) == 1
    c = planned.crossings[0]
    assert c.crossing_type is CrossingType.UNKNOWN and c.crossed_road == 'major'
    # Junction at s = 15, +/- junction_half_width (10 m).
    assert (c.start_s, c.end_s) == (pytest.approx(5.0), pytest.approx(25.0))


def test_junction_half_width_is_used():
    planned = _plan(_graph(detour=None), PlannerConfig(junction_half_width=4.0))
    c = planned.crossings[0]
    assert (c.start_s, c.end_s) == (pytest.approx(11.0), pytest.approx(19.0))


def test_invalid_junction_half_width_rejected():
    with pytest.raises(ValueError):
        PlannerConfig(junction_half_width=0.0)