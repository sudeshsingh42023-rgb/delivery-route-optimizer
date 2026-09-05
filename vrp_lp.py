"""
Linear-programming formulation of the Capacitated Vehicle Routing Problem
(CVRP), solved with PuLP (CBC solver under the hood).

This is a mixed-integer program (MIP), the standard way optimization
mathematics is applied to real routing problems (this is essentially a
small version of what logistics/operations-research teams run in
production):

    minimize   sum_{i,j} dist[i,j] * x[i,j]
    subject to:
        - each customer has exactly one outgoing and one incoming edge
        - the depot has exactly K outgoing / K incoming edges (K vehicles)
        - MTZ (Miller-Tucker-Zemlin) constraints eliminate sub-tours and
          simultaneously enforce the vehicle capacity constraint via a
          cumulative-load variable u[i]

MIP solving is exponential in the worst case, so this is only practical for
small instances (roughly <= 15-20 stops). For larger instances, use the
2-opt heuristic in heuristic.py instead -- the two are meant to be compared
in main.py to show the classic exact-vs-heuristic optimization trade-off.
"""

import math

import pulp

from .graph import Point


def solve_cvrp_lp(points: list[Point], dist: dict, depot_id: str,
                   vehicle_capacity: int, num_vehicles: int = None,
                   time_limit_seconds: int = 20):
    customers = [p for p in points if p.id != depot_id]
    ids = [depot_id] + [p.id for p in customers]
    demand = {p.id: p.demand for p in points}
    demand[depot_id] = 0

    total_demand = sum(demand[c.id] for c in customers)
    if num_vehicles is None:
        num_vehicles = max(1, math.ceil(total_demand / vehicle_capacity))

    prob = pulp.LpProblem("CVRP", pulp.LpMinimize)

    # x[i,j] = 1 if the route travels directly from i to j
    x = {
        (i, j): pulp.LpVariable(f"x_{i}_{j}", cat="Binary")
        for i in ids for j in ids if i != j
    }
    # u[i] = cumulative load delivered by the time the vehicle leaves stop i
    # (also serves as the MTZ subtour-elimination potential function)
    u = {
        i: pulp.LpVariable(f"u_{i}", lowBound=demand[i], upBound=vehicle_capacity)
        for i in customers_ids(customers)
    }

    prob += pulp.lpSum(dist[(i, j)] * x[(i, j)] for (i, j) in x)

    # Each customer: exactly one outgoing edge, exactly one incoming edge.
    for c in customers:
        prob += pulp.lpSum(x[(c.id, j)] for j in ids if j != c.id) == 1
        prob += pulp.lpSum(x[(i, c.id)] for i in ids if i != c.id) == 1

    # Depot: exactly K outgoing and K incoming edges (K vehicles depart/return).
    prob += pulp.lpSum(x[(depot_id, j)] for j in ids if j != depot_id) == num_vehicles
    prob += pulp.lpSum(x[(i, depot_id)] for i in ids if i != depot_id) == num_vehicles

    # MTZ subtour elimination + capacity: if x[i,j]=1 then u[j] >= u[i] + demand[j]
    # (standard big-M linearization). This also guarantees no route's
    # cumulative load exceeds vehicle_capacity, since u is capped above.
    for c1 in customers:
        for c2 in customers:
            if c1.id == c2.id:
                continue
            prob += (
                u[c1.id] - u[c2.id] + vehicle_capacity * x[(c1.id, c2.id)]
                <= vehicle_capacity - demand[c2.id]
            )

    solver = pulp.PULP_CBC_CMD(msg=False, timeLimit=time_limit_seconds)
    prob.solve(solver)

    status = pulp.LpStatus[prob.status]
    edges_used = [(i, j) for (i, j) in x if pulp.value(x[(i, j)]) > 0.5]
    routes = _edges_to_routes(edges_used, depot_id)
    objective = pulp.value(prob.objective)
    return {
        "status": status,
        "objective": objective,
        "routes": routes,
        "num_vehicles": num_vehicles,
    }


def customers_ids(customers):
    return [c.id for c in customers]


def _edges_to_routes(edges: list[tuple], depot_id: str) -> list[list[str]]:
    """Reconstruct individual vehicle routes by following edges out of the depot."""
    outgoing = {}
    for i, j in edges:
        outgoing.setdefault(i, []).append(j)

    routes = []
    for start in outgoing.get(depot_id, []):
        route = [depot_id, start]
        current = start
        while current != depot_id:
            nxt_options = outgoing.get(current, [])
            nxt = nxt_options[0] if nxt_options else depot_id
            if nxt == depot_id:
                break
            route.append(nxt)
            current = nxt
        routes.append(route)
    return routes
