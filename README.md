# Delivery Route Optimization Engine

Solves the **Capacitated Vehicle Routing Problem (CVRP)** three ways —
greedy, greedy + local search, and an exact linear-programming
formulation — and compares them on solution quality vs. runtime. Modeled
on the kind of last-mile delivery routing problem logistics/operations
teams solve daily.

## Why this project

Most student projects skip optimization mathematics entirely. This one
puts **linear/integer programming** front and center alongside classic
graph algorithms, directly demonstrating "experience in optimization
mathematics such as linear programming" rather than just claiming it.

## Key concepts implemented

| Concept | Where | Why it matters |
|---|---|---|
| Dijkstra's algorithm | `optimizer/graph.py` | Shortest path between any two delivery points, O((V+E) log V) |
| Greedy nearest-neighbor construction | `optimizer/heuristic.py` | Fast initial solution, capacity-aware |
| 2-opt local search | `optimizer/heuristic.py` | Removes crossing edges to improve the greedy solution |
| Mixed-integer linear program (MILP) with MTZ subtour elimination | `optimizer/vrp_lp.py` | Exact optimal solution for small instances via PuLP/CBC |
| Complexity trade-off analysis | `main.py` | Compares exact vs. heuristic: quality vs. runtime |

## Run it

```bash
pip install -r requirements.txt
python main.py
```

The demo:
1. Generates 10 random delivery stops around a depot with random demand
2. Solves the routing problem via greedy, greedy+2-opt, and exact LP/MIP
3. Prints a comparison table (distance, runtime, vehicles used)
4. Saves route visualizations as PNGs (`greedy_routes.png`, `two_opt_routes.png`, `lp_routes.png`)

## Notes on scaling

The exact LP/MIP solver is NP-hard in the worst case and only tractable for
small instances (roughly ≤15-20 stops with the given time limit). For larger
instances, the 2-opt heuristic is the practical choice — this trade-off
(exact-but-slow vs. approximate-but-fast) is exactly what the demo is meant
to illustrate, and is worth mentioning explicitly if asked about scalability
in an interview.

## Possible resume bullet

> Designed a delivery route optimization engine solving a capacitated
> vehicle routing problem via Dijkstra's algorithm, a 2-opt local-search
> heuristic, and an exact MILP formulation (PuLP/CBC with MTZ subtour
> elimination), quantifying the quality-vs-runtime trade-off between exact
> and heuristic approaches across randomized delivery instances.
