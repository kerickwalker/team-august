# plot_trajectory.py — Visualise trajectory.csv
#
# Usage:
#   python3 plot_trajectory.py                        # plots trajectory.csv
#   python3 plot_trajectory.py -i my_trajectory.csv
#   python3 plot_trajectory.py -i traj.csv -w         # also plot waypoints
#
# Shows:
#   - Path coloured by heading angle
#   - Heading arrows at regular intervals
#   - Start (green) and end (red) markers
#   - Optionally overlays teleop_waypoints.csv capture points

import argparse
import csv
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.cm as cm
from matplotlib.colors import Normalize


def load_csv(path, required=True):
    rows = []
    try:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                rows.append({k: float(v) for k, v in row.items()})
    except FileNotFoundError:
        if required:
            print(f"ERROR: file not found: '{path}'")
            sys.exit(1)
        return None
    return rows


def main():
    parser = argparse.ArgumentParser(description="Plot trajectory CSV")
    parser.add_argument("-i", "--input",     default="trajectory.csv",
                        help="Trajectory CSV to plot (default: trajectory.csv)")
    parser.add_argument("-w", "--waypoints", action="store_true",
                        help="Overlay teleop_waypoints.csv capture points")
    parser.add_argument("--wp-file",         default="teleop_waypoints.csv",
                        help="Waypoint file to overlay (default: teleop_waypoints.csv)")
    args = parser.parse_args()

    rows = load_csv(args.input)
    xs   = np.array([r["x"]               for r in rows])
    ys   = np.array([r["y"]               for r in rows])
    hdg  = np.array([r["heading"]         for r in rows])
    dist = np.array([r["cumulative_dist"]  for r in rows])

    # ── Figure ────────────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 6))
    ax.set_aspect("equal")
    ax.grid(True, linestyle="--", alpha=0.4)
    ax.set_xlabel("x  (forward, m)")
    ax.set_ylabel("y  (right, m)")
    ax.set_title(f"Trajectory: {args.input}  —  {len(rows)} pts, {dist[-1]:.2f} m")

    # Path coloured by heading
    norm   = Normalize(vmin=hdg.min(), vmax=hdg.max())
    cmap   = cm.plasma
    points = np.array([xs, ys]).T.reshape(-1, 1, 2)
    segs   = np.concatenate([points[:-1], points[1:]], axis=1)
    from matplotlib.collections import LineCollection
    lc = LineCollection(segs, cmap=cmap, norm=norm, linewidth=2, zorder=2)
    lc.set_array(hdg[:-1])
    ax.add_collection(lc)
    fig.colorbar(lc, ax=ax, label="heading (rad, CCW+)")

    # Heading arrows every ~0.5 m
    step = max(1, int(0.5 / (dist[-1] / len(dist))))
    arrow_idx = np.arange(0, len(xs), step)
    arrow_len = dist[-1] * 0.015
    for i in arrow_idx:
        ax.annotate("",
            xy=(xs[i] + arrow_len * np.cos(hdg[i]),
                ys[i] + arrow_len * np.sin(hdg[i])),
            xytext=(xs[i], ys[i]),
            arrowprops=dict(arrowstyle="->", color="steelblue", lw=1.0),
            zorder=3)

    # Start / end markers
    ax.plot(xs[0],  ys[0],  "go", markersize=10, label="start", zorder=4)
    ax.plot(xs[-1], ys[-1], "rs", markersize=10, label="end",   zorder=4)

    # Optional waypoint overlay
    if args.waypoints:
        wp = load_csv(args.wp_file, required=False)
        if wp:
            wx = [r["x"] for r in wp]
            wy = [r["y"] for r in wp]
            ax.scatter(wx, wy, c="orange", s=80, zorder=5,
                       label=f"waypoints ({len(wp)})", edgecolors="black", linewidths=0.5)
            for k, (x, y) in enumerate(zip(wx, wy)):
                ax.annotate(str(k), (x, y), textcoords="offset points",
                            xytext=(4, 4), fontsize=7)
        else:
            print(f"% WARNING: waypoint file '{args.wp_file}' not found — skipping overlay")

    ax.legend(loc="best")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
