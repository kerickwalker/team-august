#!/usr/bin/env python3
"""
plot_kalman_log.py  —  Interactive plotter for kalman_log.jsonl files.

Usage:
    python3 plot_kalman_log.py recordings/<session>/kalman_log.jsonl
    python3 plot_kalman_log.py recordings/<session>/kalman_log.jsonl --fields x.x x.y x.yaw
    python3 plot_kalman_log.py recordings/<session>/kalman_log.jsonl --xy   (2-D trajectory)
    python3 plot_kalman_log.py recordings/<session>/kalman_log.jsonl --list  (list available fields)
"""

import argparse
import json
import sys
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


# ---------------------------------------------------------------------------
# Flatten a nested dict with dot-separated keys
# ---------------------------------------------------------------------------
def flatten(d, prefix=""):
    out = {}
    for k, v in d.items():
        key = f"{prefix}.{k}" if prefix else k
        if isinstance(v, dict):
            out.update(flatten(v, key))
        elif isinstance(v, (int, float, bool)) or v is None:
            out[key] = float(v) if v is not None else float("nan")
    return out


# ---------------------------------------------------------------------------
# Load log file
# ---------------------------------------------------------------------------
def load(path):
    records = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue
            t    = obj.get("t", float("nan"))
            vt   = obj.get("video_t")           # "0:00:12.304" or null
            data = obj.get("data", {})
            flat = flatten(data)
            flat["__t"] = t
            flat["__video_t"] = _parse_video_t(vt)
            records.append(flat)
    return records


def _parse_video_t(vt):
    """Convert '0:00:12.304' → 12.304 seconds, None → nan."""
    if vt is None:
        return float("nan")
    try:
        parts = vt.split(":")
        if len(parts) == 3:
            return int(parts[0]) * 3600 + int(parts[1]) * 60 + float(parts[2])
        return float(vt)
    except Exception:
        return float("nan")


# ---------------------------------------------------------------------------
# Build time axis (prefer video_t when available, fallback to wall-clock t)
# ---------------------------------------------------------------------------
def build_time(records):
    vt = np.array([r["__video_t"] for r in records])
    if np.isfinite(vt).sum() > len(records) * 0.5:
        return vt, "video time (s)"
    t0 = min(r["__t"] for r in records if np.isfinite(r["__t"]))
    wt = np.array([r["__t"] - t0 for r in records])
    return wt, "elapsed wall-clock (s)"


# ---------------------------------------------------------------------------
# Collect available field names across all records
# ---------------------------------------------------------------------------
def available_fields(records):
    fields = set()
    for r in records:
        fields.update(k for k in r if not k.startswith("__"))
    return sorted(fields)


# ---------------------------------------------------------------------------
# Extract a named field as a numpy array aligned to time axis
# ---------------------------------------------------------------------------
def extract(records, field):
    return np.array([r.get(field, float("nan")) for r in records])


# ---------------------------------------------------------------------------
# Default fields to plot when none are specified
# ---------------------------------------------------------------------------
DEFAULT_FIELDS = [
    ["x.x", "x.y"],
    ["x.yaw", "x.pitch"],
    ["x.velocity", "x.angular_velocity"],
    ["x.z"],
    ["measurements.ahrs_pitch", "measurements.ahrs_roll", "measurements.ahrs_yaw"],
]


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("log", help="Path to kalman_log.jsonl")
    ap.add_argument("--fields", nargs="+", help="Dot-path fields to plot (each gets its own subplot)")
    ap.add_argument("--together", nargs="+", help="Plot these fields on a single subplot")
    ap.add_argument("--xy", action="store_true", help="2-D trajectory plot (x.x vs x.y)")
    ap.add_argument("--list", action="store_true", help="List all available fields and exit")
    args = ap.parse_args()

    path = Path(args.log)
    if not path.exists():
        print(f"File not found: {path}", file=sys.stderr)
        sys.exit(1)

    records = load(path)
    if not records:
        print("No records found.", file=sys.stderr)
        sys.exit(1)

    if args.list:
        for f in available_fields(records):
            print(f)
        return

    time, time_label = build_time(records)

    # --- 2-D trajectory ---
    if args.xy:
        x = extract(records, "x.x")
        y = extract(records, "x.y")
        fig, ax = plt.subplots(figsize=(7, 6))
        sc = ax.scatter(x, y, c=time, cmap="viridis", s=4)
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title("2-D Trajectory")
        ax.set_aspect("equal")
        plt.colorbar(sc, ax=ax, label=time_label)
        plt.tight_layout()
        plt.show()
        return

    # --- Field plots ---
    if args.together:
        groups = [args.together]
    elif args.fields:
        groups = [[f] for f in args.fields]
    else:
        # filter default groups to only those with at least one valid field
        afields = set(available_fields(records))
        groups = [[f for f in g if f in afields] for g in DEFAULT_FIELDS]
        groups = [g for g in groups if g]

    n = len(groups)
    if n == 0:
        print("No plottable fields found.")
        return

    fig, axes = plt.subplots(n, 1, figsize=(12, 2.8 * n), sharex=True)
    if n == 1:
        axes = [axes]

    for ax, group in zip(axes, groups):
        for field in group:
            vals = extract(records, field)
            mask = np.isfinite(time) & np.isfinite(vals)
            if mask.sum() == 0:
                continue
            ax.plot(time[mask], vals[mask], linewidth=0.8, label=field)
        ax.set_ylabel(", ".join(group), fontsize=8)
        ax.legend(fontsize=7, loc="upper right")
        ax.grid(True, alpha=0.3)

    axes[-1].set_xlabel(time_label)
    fig.suptitle(path.name, fontsize=10)
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
