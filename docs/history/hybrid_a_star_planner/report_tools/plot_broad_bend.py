#!/usr/bin/env python3
"""Render the deterministic channel bend regression data; not an RViz capture."""
import argparse
import csv
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input-dir", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    fig, axes = plt.subplots(2, 1, figsize=(10, 7), constrained_layout=True)
    x = np.linspace(-45, 80, 600)
    y = np.tan(np.deg2rad(20)) * np.maximum(x, 0)
    axes[0].fill_between(x, y - 12, y + 12, color="#eaf4ea", label="Main lane (24m wide)")
    axes[0].plot(x, y - 12, color="#54a154", linewidth=1)
    axes[0].plot(x, y + 12, color="#54a154", linewidth=1)
    axes[0].plot(x, y, "--", color="#649664", label="Lane center")
    for name, label, color in (("left_before", "Before", "#d87832"),
                               ("left_after", "After", "#1573a3")):
        with (args.input_dir / (name + ".csv")).open() as stream:
            rows = list(csv.DictReader(stream))
        xx = np.array([float(row["x"]) for row in rows])
        yy = np.array([float(row["y"]) for row in rows])
        k = np.array([float(row["curvature"]) for row in rows])
        axes[0].plot(xx, yy, color=color, linewidth=2, label=label)
        axes[1].plot(xx, k, color=color, linewidth=1.7, label=label)
    axes[0].set_aspect("equal")
    axes[0].set_ylabel("y (m)")
    axes[0].set_title("20-degree channel bend: same vessel model, same 8m minimum-radius parameter")
    axes[1].set_xlabel("x (m)")
    axes[1].set_ylabel("Ground-track curvature (1/m)")
    for axis in axes:
        axis.grid(alpha=0.2)
        axis.legend(loc="upper left")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.output, dpi=150)


if __name__ == "__main__":
    main()
