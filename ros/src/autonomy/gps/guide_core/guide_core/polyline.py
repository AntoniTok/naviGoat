"""Polyline simplification (Ramer-Douglas-Peucker) for planned routes.

Removes points that add nothing to a route's shape while guaranteeing that
every original point stays within `tolerance` of the simplified line.
"""

import math


def point_segment_distance(p, a, b):
    """Return the distance from point p to the line segment a-b (not the infinite line).

    All arguments are (x, y) pairs. If a == b, this is the distance from p to a.
    """
    px, py = p
    ax, ay = a
    bx, by = b
    dx, dy = bx - ax, by - ay
    seg_sq = dx * dx + dy * dy
    if seg_sq == 0.0:
        return math.hypot(px - ax, py - ay)
    # Position of p's projection along a-b: 0 at a, 1 at b; clamped to the segment.
    t = ((px - ax) * dx + (py - ay) * dy) / seg_sq
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def simplify_polyline(points, tolerance, keep=()):
    """Return the sorted indices of the points to keep.

    points: sequence of (x, y). tolerance: max allowed distance (metres) from
    any original point to the simplified line; finite and >= 0.
    keep: indices that must be kept (e.g. crossing kerb points). The first
    and last points are always kept. Points exactly within tolerance are dropped.

    Raises ValueError for an invalid tolerance or keep index.
    """
    n = len(points)
    tolerance = float(tolerance)
    if not (math.isfinite(tolerance) and tolerance >= 0.0):
        raise ValueError(f'tolerance must be finite and >= 0, got {tolerance!r}')
    protected = set()
    for i in keep:
        if isinstance(i, bool) or not isinstance(i, int) or not 0 <= i < n:
            raise ValueError(f'keep index {i!r} is not an int in [0, {n - 1}]')
        protected.add(i)
    if n <= 2:
        return list(range(n))

    # Protected points split the line into pieces; each piece is simplified
    # on its own, so protected points are never dropped.
    anchors = sorted(protected | {0, n - 1})
    kept = set(anchors)
    for start, end in zip(anchors, anchors[1:]):
        # A stack instead of recursion: long routes can't hit Python's recursion limit.
        stack = [(start, end)]
        while stack:
            s, e = stack.pop()
            if e - s < 2:
                continue
            worst_i, worst_d = s + 1, -1.0
            for i in range(s + 1, e):
                d = point_segment_distance(points[i], points[s], points[e])
                if d > worst_d:
                    worst_i, worst_d = i, d
            if worst_d > tolerance:
                kept.add(worst_i)
                stack.append((s, worst_i))
                stack.append((worst_i, e))
    return sorted(kept)
