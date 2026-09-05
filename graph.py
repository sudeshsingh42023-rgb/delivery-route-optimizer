"""
Graph construction + shortest-path algorithms.

Delivery points are modelled as nodes in a complete graph, weighted by
Euclidean distance (a proxy for road distance). Dijkstra's algorithm gives
the shortest path between any two points, which the route optimizer uses
as its distance metric.
"""

import heapq
import math
from dataclasses import dataclass


@dataclass
class Point:
    id: str
    x: float
    y: float
    demand: int = 0  # units of cargo required at this stop (0 for depot)


def euclidean(a: Point, b: Point) -> float:
    return math.hypot(a.x - b.x, a.y - b.y)


def build_distance_matrix(points: list[Point]) -> dict[tuple[str, str], float]:
    """O(n^2) — fine for delivery-sized inputs (tens to low hundreds of stops)."""
    dist = {}
    for a in points:
        for b in points:
            dist[(a.id, b.id)] = euclidean(a, b)
    return dist


def dijkstra(points: list[Point], dist: dict, source_id: str) -> dict[str, float]:
    """
    Standard Dijkstra using a binary heap: O((V + E) log V).
    Here the graph is complete (E = V^2) since any two delivery points are
    "reachable" directly, but the algorithm generalizes to sparse road
    networks where edges only exist between adjacent intersections.
    """
    ids = [p.id for p in points]
    shortest = {i: math.inf for i in ids}
    shortest[source_id] = 0
    visited = set()
    heap = [(0, source_id)]

    while heap:
        d, u = heapq.heappop(heap)
        if u in visited:
            continue
        visited.add(u)
        for v in ids:
            if v == u or v in visited:
                continue
            weight = dist[(u, v)]
            if d + weight < shortest[v]:
                shortest[v] = d + weight
                heapq.heappush(heap, (shortest[v], v))
    return shortest


def route_distance(route: list[str], dist: dict) -> float:
    """Total distance of a route visiting stops in order, returning to start."""
    total = 0.0
    for i in range(len(route)):
        a = route[i]
        b = route[(i + 1) % len(route)]
        total += dist[(a, b)]
    return total
