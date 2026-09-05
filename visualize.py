"""Plot delivery points and routes with matplotlib, saved to a PNG file."""

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from .graph import Point


def plot_routes(points: list[Point], routes: list[list[str]], depot_id: str,
                 title: str, out_path: str):
    coords = {p.id: (p.x, p.y) for p in points}
    fig, ax = plt.subplots(figsize=(7, 6))

    colors = plt.cm.tab10.colors
    for idx, route in enumerate(routes):
        xs = [coords[stop][0] for stop in route] + [coords[route[0]][0]]
        ys = [coords[stop][1] for stop in route] + [coords[route[0]][1]]
        ax.plot(xs, ys, "-o", color=colors[idx % len(colors)], label=f"Vehicle {idx + 1}")

    for p in points:
        marker = "s" if p.id == depot_id else "o"
        ax.scatter(p.x, p.y, c="black" if p.id == depot_id else None,
                   marker=marker, zorder=5, s=60 if p.id == depot_id else 20)
        ax.annotate(p.id, (p.x, p.y), fontsize=7, xytext=(3, 3), textcoords="offset points")

    ax.set_title(title)
    ax.legend(loc="best", fontsize=8)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    fig.tight_layout()
    fig.savefig(out_path, dpi=150)
    plt.close(fig)
