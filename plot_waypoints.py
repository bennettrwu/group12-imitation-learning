"""
plot_waypoints.py
Plots waypoints from a CSV file (columns: latitude, longitude, heading, ...).
Draws the path as a line, marks start/end, and shows heading arrows.

Usage:
    python plot_waypoints.py wps.csv
    python plot_waypoints.py wps.csv --arrow-step 50  # heading arrow every N points
"""

import sys
import argparse
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches

# ── CLI ──────────────────────────────────────────────────────────────────────
parser = argparse.ArgumentParser(description="Plot waypoints with heading arrows.")
parser.add_argument("csv_file", help="Path to the waypoint CSV file")
parser.add_argument(
    "--arrow-step",
    type=int,
    default=50,
    metavar="N",
    help="Draw a heading arrow every N waypoints (default: 50)",
)
args = parser.parse_args()

# ── Load data ────────────────────────────────────────────────────────────────
data = np.loadtxt(args.csv_file, delimiter=",", usecols=(0, 1, 2))
lat     = data[:, 0]   # x-axis
lon     = data[:, 1]   # y-axis
heading = data[:, 2]   # radians

# ── Plot ─────────────────────────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(10, 7))

# Path
ax.plot(lat, lon, color="steelblue", linewidth=1.2, zorder=1, label="Path")

# Start / end markers
ax.scatter(lat[0],  lon[0],  color="green", s=80, zorder=3, label="Start")
ax.scatter(lat[-1], lon[-1], color="red",   s=80, zorder=3, label="End")

# Heading arrows (heading is the angle from the +x axis in radians)
arrow_scale = (np.ptp(lat) + np.ptp(lon)) / 2 * 0.025  # ~2.5 % of plot span
indices = range(0, len(lat), args.arrow_step)
for i in indices:
    dx = np.cos(heading[i]) * arrow_scale
    dy = np.sin(heading[i]) * arrow_scale
    ax.annotate(
        "",
        xy=(lat[i] + dx, lon[i] + dy),
        xytext=(lat[i], lon[i]),
        arrowprops=dict(arrowstyle="-|>", color="darkorange", lw=1.2),
        zorder=2,
    )

# Dummy handle for the legend
arrow_patch = mpatches.Patch(color="darkorange", label=f"Heading (every {args.arrow_step} pts)")

ax.set_xlabel("Latitude")
ax.set_ylabel("Longitude")
ax.set_title("Waypoint Path with Headings")
ax.legend(handles=[
    plt.Line2D([0], [0], color="steelblue", lw=1.5, label="Path"),
    plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="green",
               markersize=8, label="Start"),
    plt.Line2D([0], [0], marker="o", color="w", markerfacecolor="red",
               markersize=8, label="End"),
    arrow_patch,
])
ax.grid(True, linestyle="--", alpha=0.4)
ax.set_aspect("equal", adjustable="datalim")

plt.tight_layout()
plt.savefig("waypoints.png", dpi=150)
plt.show()
print("Plot saved to waypoints.png")
