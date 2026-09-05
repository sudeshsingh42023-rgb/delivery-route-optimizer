"""
Demo: generates a set of delivery points around a depot, then solves the
Capacitated Vehicle Routing Problem three ways so the trade-offs are
visible side by side:

  1. Greedy nearest-neighbor construction (fast, weakest quality)
  2. Greedy + 2-opt local search (fast, noticeably better quality)
  3. Exact LP / MIP formulation via PuLP (slowest, optimal for small n)

Run with:  python main.py
"""

import random
import time

from optimizer import (
    Point, build_distance_matrix, route_distance,
    nearest_neighbor_routes, two_opt, solve_cvrp_lp,
)
from optimizer.visualize import plot_routes


def generate_points(n_customers: int, seed: int = 42) -> list[Point]:
    random.seed(seed)
    points = [Point(id="depot", x=50, y=50, demand=0)]
    for i in range(n_customers):
        points.append(Point(
            id=f"C{i+1}",
            x=random.uniform(0, 100),
            y=random.uniform(0, 100),
            demand=random.randint(5, 15),
        ))
    return points


def total_distance(routes: list[list[str]], dist: dict) -> float:
    return sum(route_distance(r, dist) for r in routes)


def main():
    depot_id = "depot"
    vehicle_capacity = 40

    points = generate_points(n_customers=10)
    dist = build_distance_matrix(points)

    print(f"Generated {len(points) - 1} delivery stops, vehicle capacity = {vehicle_capacity}\n")

    # 1. Greedy nearest-neighbor baseline.
    t0 = time.time()
    greedy_routes = nearest_neighbor_routes(points, dist, depot_id, vehicle_capacity)
    greedy_time = time.time() - t0
    greedy_dist = total_distance(greedy_routes, dist)

    # 2. Greedy + 2-opt local search.
    t0 = time.time()
    two_opt_routes = [two_opt(r, dist) for r in greedy_routes]
    two_opt_time = time.time() - t0 + greedy_time
    two_opt_dist = total_distance(two_opt_routes, dist)

    # 3. Exact LP / MIP formulation (small instance, so this is tractable).
    t0 = time.time()
    lp_result = solve_cvrp_lp(points, dist, depot_id, vehicle_capacity, time_limit_seconds=20)
    lp_time = time.time() - t0

    print(f"{'Method':<28}{'Distance':>12}{'Time (s)':>12}{'Vehicles':>12}")
    print("-" * 64)
    print(f"{'Greedy nearest-neighbor':<28}{greedy_dist:>12.2f}{greedy_time:>12.3f}{len(greedy_routes):>12}")
    print(f"{'Greedy + 2-opt':<28}{two_opt_dist:>12.2f}{two_opt_time:>12.3f}{len(two_opt_routes):>12}")
    print(f"{'Exact LP/MIP (' + lp_result['status'] + ')':<28}{lp_result['objective']:>12.2f}{lp_time:>12.3f}{lp_result['num_vehicles']:>12}")

    improvement = 100 * (greedy_dist - two_opt_dist) / greedy_dist
    print(f"\n2-opt improved on greedy by {improvement:.1f}%")
    if lp_result["objective"]:
        gap = 100 * (two_opt_dist - lp_result["objective"]) / lp_result["objective"]
        print(f"2-opt is {gap:.1f}% above the LP-optimal solution")

    plot_routes(points, greedy_routes, depot_id, "Greedy Nearest-Neighbor", "greedy_routes.png")
    plot_routes(points, two_opt_routes, depot_id, "Greedy + 2-opt", "two_opt_routes.png")
    plot_routes(points, lp_result["routes"], depot_id, "Exact LP/MIP", "lp_routes.png")
    print("\nSaved route plots: greedy_routes.png, two_opt_routes.png, lp_routes.png")


if __name__ == "__main__":
    main()
