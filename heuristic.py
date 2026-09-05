"""
Heuristic solvers for the Capacitated Vehicle Routing Problem (CVRP).

The CVRP is NP-hard, so exact solutions (see vrp_lp.py) only scale to
small instances. For larger instances we fall back to:

  1. Greedy nearest-neighbor construction, respecting vehicle capacity
     (splits stops into multiple trips once capacity is exceeded).
  2. 2-opt local search to remove crossing edges within each route,
     a standard local-improvement step for TSP/VRP.

Complexity: nearest-neighbor construction is O(n^2); 2-opt is O(n^2) per
pass and is run until no improving swap is found (bounded number of passes
in practice).
"""

from .graph import Point, route_distance


def nearest_neighbor_routes(points: list[Point], dist: dict, depot_id: str,
                             vehicle_capacity: int) -> list[list[str]]:
    """Greedy construction: repeatedly visit the nearest unvisited stop that
    still fits in the current vehicle's remaining capacity; start a new
    vehicle/route when it doesn't."""
    stops = {p.id: p for p in points if p.id != depot_id}
    unvisited = set(stops.keys())
    routes = []

    while unvisited:
        route = [depot_id]
        capacity_left = vehicle_capacity
        current = depot_id
        while True:
            candidates = [
                sid for sid in unvisited
                if stops[sid].demand <= capacity_left
            ]
            if not candidates:
                break
            nxt = min(candidates, key=lambda sid: dist[(current, sid)])
            route.append(nxt)
            capacity_left -= stops[nxt].demand
            unvisited.discard(nxt)
            current = nxt
        routes.append(route)

    return routes


def two_opt(route: list[str], dist: dict) -> list[str]:
    """
    Classic 2-opt: repeatedly reverse a segment of the route if doing so
    shortens total distance, until no such improving move exists.
    Removes the "crossing path" inefficiency that greedy construction
    often leaves behind.
    """
    best = route[:]
    improved = True
    while improved:
        improved = False
        for i in range(1, len(best) - 1):
            for j in range(i + 1, len(best)):
                if j - i == 1:
                    continue
                new_route = best[:i] + best[i:j][::-1] + best[j:]
                if route_distance(new_route, dist) < route_distance(best, dist):
                    best = new_route
                    improved = True
    return best


def solve_heuristic(points: list[Point], dist: dict, depot_id: str,
                     vehicle_capacity: int) -> list[list[str]]:
    routes = nearest_neighbor_routes(points, dist, depot_id, vehicle_capacity)
    return [two_opt(r, dist) for r in routes]
