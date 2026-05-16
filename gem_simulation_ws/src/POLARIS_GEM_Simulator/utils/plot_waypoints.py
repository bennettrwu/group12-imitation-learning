#!/usr/bin/env python3

import argparse
import csv
import math
import os
import sys

import matplotlib.pyplot as plt


def load(path):
    xs, ys, yaws = [], [], []
    with open(path) as f:
        for row in csv.reader(f):
            if not row:
                continue
            xs.append(float(row[0]))
            ys.append(float(row[1]))
            yaws.append(float(row[2]))
    return xs, ys, yaws


def plot(xs, ys, yaws, title, out_path=None):
    fig, ax = plt.subplots(figsize=(10, 8))
    ax.plot(xs, ys, "-", color="steelblue", lw=1, label="path")
    sc = ax.scatter(xs, ys, s=8, c=range(len(xs)), cmap="viridis")
    ax.scatter(
        [xs[0]], [ys[0]], s=120, c="lime", edgecolors="black", zorder=5, label="start"
    )
    ax.scatter(
        [xs[-1]], [ys[-1]], s=120, c="red", edgecolors="black", zorder=5, label="end"
    )

    step = max(1, len(xs) // 40)
    for i in range(0, len(xs), step):
        ax.arrow(
            xs[i],
            ys[i],
            0.6 * math.cos(yaws[i]),
            0.6 * math.sin(yaws[i]),
            head_width=0.25,
            head_length=0.25,
            fc="black",
            ec="black",
            alpha=0.6,
        )

    cbar = plt.colorbar(sc, ax=ax)
    cbar.set_label("waypoint index")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(title)
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    ax.legend(loc="best")
    plt.tight_layout()

    if out_path:
        plt.savefig(out_path, dpi=120)
        print(f"saved {out_path}")
    else:
        plt.show()


def main():
    default_csv = os.path.join(
        os.path.dirname(os.path.abspath(__file__)), "waypoints", "waypoints.csv"
    )

    p = argparse.ArgumentParser(description="Plot a waypoints CSV (x, y, yaw per row).")
    p.add_argument(
        "csv",
        nargs="?",
        default=default_csv,
        help=f"path to waypoints CSV (default: {default_csv})",
    )
    p.add_argument(
        "-o", "--out", default="plot.png", help="filename to save (default: plot.png)"
    )
    args = p.parse_args()

    if not os.path.exists(args.csv):
        print(f"error: {args.csv} not found", file=sys.stderr)
        sys.exit(1)

    xs, ys, yaws = load(args.csv)
    if not xs:
        print(f"error: {args.csv} is empty", file=sys.stderr)
        sys.exit(1)

    title = (
        f"{os.path.basename(args.csv)}  —  {len(xs)} waypoints  "
        f"x:[{min(xs):.1f},{max(xs):.1f}]  y:[{min(ys):.1f},{max(ys):.1f}]"
    )
    plot(xs, ys, yaws, title, args.out)


if __name__ == "__main__":
    main()
