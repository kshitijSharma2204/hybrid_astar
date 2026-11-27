import numpy as np
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
from matplotlib.patches import Polygon

FORWARD = "#1f5fbf"
REVERSE = "#d0342c"
RAW = "#888888"


def car_polygon(x, y, theta, length, width, rear_axle_to_center=0.0):
    """Corners of the car rectangle as a (4, 2) array.

    (x, y) is the rear axle; the rectangle's centre is rear_axle_to_center
    ahead of it, the same convention as the planner.
    """
    c, s = np.cos(theta), np.sin(theta)
    d = rear_axle_to_center
    local = np.array([[d + length / 2, width / 2], [d + length / 2, -width / 2],
                      [d - length / 2, -width / 2], [d - length / 2, width / 2]])
    return local @ np.array([[c, s], [-s, c]]) + [x, y]


def draw_map(ax, grid):
    data = grid.data
    img = np.where(data >= 100, 0.15, 1.0)
    ax.imshow(img, cmap="gray", vmin=0, vmax=1, origin="lower",
              extent=grid.extent, interpolation="nearest")
    ax.set_xlim(grid.extent[0], grid.extent[1])
    ax.set_ylim(grid.extent[2], grid.extent[3])
    ax.set_aspect("equal")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")


def path_segments(x, y, direction):
    """Split a path into runs of equal direction: [(xs, ys, dir), ...]."""
    runs = []
    start = 0
    for i in range(1, len(x) + 1):
        if i == len(x) or direction[i] != direction[start]:
            # include the next point so runs join up on screen
            end = min(i + 1, len(x))
            runs.append((x[start:end], y[start:end], direction[start]))
            start = i
    return runs


def draw_path(ax, x, y, direction, lw=2.0, zorder=4):
    artists = []
    for xs, ys, d in path_segments(np.asarray(x), np.asarray(y), np.asarray(direction)):
        artists += ax.plot(xs, ys, color=FORWARD if d > 0 else REVERSE, lw=lw,
                           zorder=zorder, solid_capstyle="round")
    return artists


def draw_car(ax, x, y, theta, config, color="k", alpha=1.0, fill=False, lw=1.0, zorder=5):
    poly = Polygon(car_polygon(x, y, theta, config.vehicle_length, config.vehicle_width,
                               config.rear_axle_to_center),
                   closed=True, fill=fill, ec=color, fc=color if fill else "none",
                   alpha=alpha, lw=lw, zorder=zorder)
    ax.add_patch(poly)
    # a line from the rear axle to the front bumper shows the heading
    front = config.rear_axle_to_center + 0.5 * config.vehicle_length
    hx = x + front * np.cos(theta)
    hy = y + front * np.sin(theta)
    line, = ax.plot([x, hx], [y, hy], color=color, lw=lw, alpha=alpha, zorder=zorder)
    return [poly, line]


def plot_result(grid, result, config, start=None, goal=None, ax=None, out=None,
                title=None, car_every=4.0):
    """Static picture of one plan.

    Shows the map, the raw search path (thin grey) under the final path,
    reverse driving in red, and the car outline about every car_every meters.
    """
    own_fig = ax is None
    if own_fig:
        fig, ax = plt.subplots(figsize=(8, 8 * (grid.extent[3] - grid.extent[2]) /
                                        max(grid.extent[1] - grid.extent[0], 1e-9) + 0.6))
    draw_map(ax, grid)

    if start is not None:
        draw_car(ax, *start, config, color="#2a9d3c", lw=1.5)
    if goal is not None:
        draw_car(ax, *goal, config, color="#c2185b", lw=1.5)

    if result.success:
        raw = getattr(result, "raw_x", None)
        if raw is not None and len(raw) and len(raw) != 0:
            ax.plot(result.raw_x, result.raw_y, color=RAW, lw=0.8, zorder=3)
        draw_path(ax, result.x, result.y, result.direction)
        x, y, th = result.x, result.y, result.theta
        dist = np.concatenate([[0], np.cumsum(np.hypot(np.diff(x), np.diff(y)))])
        next_mark = 0.0
        for i in range(len(x)):
            if dist[i] >= next_mark:
                draw_car(ax, x[i], y[i], th[i], config, color="#444444", alpha=0.5, lw=0.8)
                next_mark = dist[i] + car_every

    handles = [plt.Line2D([], [], color=FORWARD, lw=2, label="forward"),
               plt.Line2D([], [], color=REVERSE, lw=2, label="reverse")]
    if getattr(result, "raw_x", None) is not None:
        handles.append(plt.Line2D([], [], color=RAW, lw=0.8, label="search path"))
    ax.legend(handles=handles, loc="upper right", fontsize=8, framealpha=0.9)
    if title:
        ax.set_title(title)

    if own_fig:
        fig.tight_layout()
        if out:
            fig.savefig(out, dpi=100)
            plt.close(fig)
        return fig
    return ax
