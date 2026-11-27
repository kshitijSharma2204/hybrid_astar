import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from matplotlib.collections import LineCollection

from .plot import draw_car, draw_map, draw_path

TREE = "#5aa9e6"
RS_HIT = "#f4a259"
RS_OK = "#2a9d3c"


def _tree_segments(nodes):
    """One segment per expanded node, from its parent to itself."""
    parent = nodes[:, 3].astype(int)
    seg = np.zeros((len(nodes), 2, 2))
    seg[:, 1, :] = nodes[:, :2]
    seg[:, 0, :] = nodes[np.maximum(parent, 0), :2]
    seg[parent < 0, 0, :] = nodes[parent < 0, :2]
    return seg


def animate_search(grid, result, config, out, start=None, goal=None, title=None,
                   width_px=800, fps=15, search_frames=90, drive_frames=60,
                   rs_hold=3, max_rs_shown=60):
    """Write a GIF or MP4 (by file extension) of the search and the result.

    The search tree grows over search_frames frames. Reeds-Shepp attempts
    show for rs_hold frames each (orange = hit an obstacle, green = used).
    Then the final path is drawn and the car drives along it.
    """
    trace = result.trace
    if trace is None:
        raise ValueError("plan with record_trace=True to animate the search")
    nodes = trace.nodes
    segs = _tree_segments(nodes)

    # Which Reeds-Shepp attempts to show, and at which expansion they happened.
    attempts = list(trace.rs_attempts)
    if len(attempts) > max_rs_shown:
        keep = set(np.linspace(0, len(attempts) - 1, max_rs_shown).astype(int))
        keep.add(len(attempts) - 1)
        attempts = [a for i, a in enumerate(attempts) if i in keep]

    ext = grid.extent
    aspect = (ext[3] - ext[2]) / (ext[1] - ext[0])
    dpi = 100
    fig_w = width_px / dpi
    fig, ax = plt.subplots(figsize=(fig_w, fig_w * aspect + 0.5), dpi=dpi)
    draw_map(ax, grid)
    if title:
        ax.set_title(title, fontsize=10)
    if start is not None:
        draw_car(ax, *start, config, color="#2a9d3c", lw=1.5)
    if goal is not None:
        draw_car(ax, *goal, config, color="#c2185b", lw=1.5)
    fig.tight_layout()

    tree = LineCollection([], colors=TREE, linewidths=0.6, zorder=2)
    ax.add_collection(tree)
    status = ax.text(0.01, 0.99, "", transform=ax.transAxes, va="top", ha="left",
                     fontsize=9, family="monospace",
                     bbox=dict(fc="white", ec="none", alpha=0.8))

    n = len(nodes)
    # Nodes shown after each search frame. Grow faster later on so the start
    # of the search, where the tree is small, is easy to follow.
    shown = np.unique(np.round(n * np.linspace(0, 1, search_frames + 1)[1:] ** 1.5).astype(int))
    shown = np.maximum(shown, 1)

    rs_lines = []         # (artist, frames_left)
    rs_queue = sorted(attempts, key=lambda a: a.node)
    rs_pos = 0

    path_frames = 10 if result.success else 0
    x, y, th = (np.asarray(result.x), np.asarray(result.y), np.asarray(result.theta))
    drive_idx = (np.linspace(0, len(x) - 1, drive_frames).astype(int)
                 if result.success and len(x) else [])
    car_artists = []
    path_drawn = []

    def frame(k):
        nonlocal rs_pos, car_artists
        if k < len(shown):
            m = shown[k]
            tree.set_segments(segs[:m])
            for a in list(rs_lines):
                a[1] -= 1
                if a[1] <= 0:
                    a[0].remove()
                    rs_lines.remove(a)
            while rs_pos < len(rs_queue) and rs_queue[rs_pos].node < m:
                at = rs_queue[rs_pos]
                s = at.samples
                if len(s):
                    color = RS_HIT if at.collided else RS_OK
                    line, = ax.plot(s[:, 0], s[:, 1], color=color, lw=1.2, zorder=3,
                                    ls="--" if at.collided else "-")
                    rs_lines.append([line, rs_hold if at.collided else 10 ** 6])
                rs_pos += 1
            status.set_text("expanded %6d / %d" % (m, n))
            return
        k -= len(shown)
        if k < path_frames:
            if k == 0:
                for a in rs_lines:
                    if a[1] < 10 ** 6:
                        a[0].remove()
                path_drawn.extend(draw_path(ax, x, y, result.direction, lw=2.5, zorder=4))
                status.set_text("path: %.1f m, %d expansions, %.0f ms" % (
                    np.sum(np.hypot(np.diff(x), np.diff(y))), n, result.stats.search_ms))
            return
        k -= path_frames
        for a in car_artists:
            a.remove()
        i = drive_idx[k]
        car_artists = draw_car(ax, x[i], y[i], th[i], config, color="#ff8c00",
                               alpha=0.8, fill=True, lw=1.0, zorder=6)

    total = len(shown) + path_frames + len(drive_idx)
    anim = animation.FuncAnimation(fig, frame, frames=total, interval=1000 / fps, blit=False)
    if str(out).endswith(".gif"):
        writer = animation.PillowWriter(fps=fps)
    else:
        writer = animation.FFMpegWriter(fps=fps, bitrate=1800)
    anim.save(out, writer=writer, dpi=dpi)
    plt.close(fig)
    return out
