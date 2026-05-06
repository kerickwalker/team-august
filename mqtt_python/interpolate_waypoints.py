# interpolate_waypoints.py — Cubic-spline resampler for teleop waypoints
#
# Converts sparse waypoints (captured with mqtt_client_teleop.py) into a
# dense trajectory.csv suitable for the Pure Pursuit controller (spursuit.py).
#
# The Pure Pursuit controller has NO internal interpolation — it walks discrete
# rows and uses cumulative_dist for lookahead.  It assumes ~0.05 m row spacing.
# Sparse waypoints (0.5-2 m apart) cause jumpy steering; this script fixes that.
#
# Usage:
#   python3 interpolate_waypoints.py                      # uses defaults
#   python3 interpolate_waypoints.py -i wps.csv -o trajectory.csv -d 0.05
#
# Output columns (same as trajectory.csv):
#   x, y, heading, cumulative_dist
#
# Coordinate convention (both input waypoints and output trajectory.csv):
#   x            Right  (+x = right)
#   y            Forward (+y = forward, robot faces +y at path start)
#   heading      CCW+    (radians, from spline tangent — not used by spursuit.py)
#   cumulative_dist  metres along the resampled path

import argparse
import csv
import math
import sys

import numpy as np


# ── Numpy-only natural cubic spline ──────────────────────────────────────────

def _make_spline(s, vals):
    """
    Build a natural cubic spline through (s, vals) using only numpy.

    Returns a callable f(t, deriv=0) that evaluates the spline value (deriv=0)
    or first derivative (deriv=1) at an array of parameter values t.

    Algorithm: solves the symmetric tridiagonal system for the interior second
    derivatives M[1..n-2] (natural boundary: M[0] = M[n-1] = 0), then
    evaluates using the standard piecewise-cubic formula.
    """
    s    = np.asarray(s,    dtype=float)
    vals = np.asarray(vals, dtype=float)
    n    = len(s)
    h    = np.diff(s)           # interval widths

    if n < 3:
        # Only 2 points — fall back to linear
        def _linear(t, deriv=0):
            t = np.asarray(t, dtype=float)
            if deriv == 0:
                return np.interp(t, s, vals)
            slope = (vals[-1] - vals[0]) / h[0] if h[0] != 0 else 0.0
            return np.full(t.shape, slope)
        return _linear

    # Right-hand side of the tridiagonal system
    rhs = 6.0 * ((vals[2:] - vals[1:-1]) / h[1:]
                - (vals[1:-1] - vals[:-2]) / h[:-1])

    # Diagonal and super/sub-diagonal
    diag = 2.0 * (h[:-1] + h[1:])
    off  = h[1:-1]

    # Build and solve (size n-2 system)
    A        = np.diag(diag) + np.diag(off, 1) + np.diag(off, -1)
    M_int    = np.linalg.solve(A, rhs)
    M        = np.concatenate([[0.0], M_int, [0.0]])   # full second-derivative vector

    def _eval(t, deriv=0):
        t   = np.asarray(t, dtype=float)
        idx = np.searchsorted(s, t, side="right") - 1
        idx = np.clip(idx, 0, n - 2)

        hi = h[idx]
        ta = s[idx + 1] - t          # distance to right knot
        tb = t - s[idx]              # distance from left knot

        if deriv == 0:
            return (M[idx]     * ta**3 / (6.0 * hi)
                  + M[idx + 1] * tb**3 / (6.0 * hi)
                  + (vals[idx]     - M[idx]     * hi**2 / 6.0) * ta / hi
                  + (vals[idx + 1] - M[idx + 1] * hi**2 / 6.0) * tb / hi)
        else:   # first derivative
            return (-M[idx]     * ta**2 / (2.0 * hi)
                  +  M[idx + 1] * tb**2 / (2.0 * hi)
                  + (vals[idx + 1] - vals[idx]) / hi
                  - (M[idx + 1] - M[idx]) * hi / 6.0)

    return _eval


# ── Waypoint helpers ──────────────────────────────────────────────────────────

def load_waypoints(path: str):
    """Read CSV waypoints → (xs, ys) numpy arrays."""
    xs, ys = [], []
    try:
        with open(path, newline="") as f:
            reader = csv.DictReader(f)
            for row in reader:
                xs.append(float(row["x"]))
                ys.append(float(row["y"]))
    except FileNotFoundError:
        print(f"% ERROR: input file not found: '{path}'")
        sys.exit(1)
    except KeyError as e:
        print(f"% ERROR: missing column {e} in '{path}'")
        sys.exit(1)
    if len(xs) < 2:
        print(f"% ERROR: need at least 2 waypoints, got {len(xs)}")
        sys.exit(1)
    return np.array(xs), np.array(ys)


def chord_cumulative(xs, ys):
    """Cumulative chord-length parameter s[i] along the waypoints."""
    s = [0.0]
    for i in range(1, len(xs)):
        dx = xs[i] - xs[i - 1]
        dy = ys[i] - ys[i - 1]
        s.append(s[-1] + math.sqrt(dx * dx + dy * dy))
    return np.array(s)


def interpolate(xs, ys, spacing: float):
    """
    Fit a natural cubic spline through (xs, ys) parameterised by chord length,
    then resample at `spacing` metre intervals.

    Returns (x_out, y_out, heading_out, cumdist_out) as numpy arrays.
    """
    s = chord_cumulative(xs, ys)

    cs_x = _make_spline(s, xs)
    cs_y = _make_spline(s, ys)

    # Dense sample points
    s_dense = np.arange(0.0, s[-1], spacing)
    if s_dense[-1] < s[-1]:
        s_dense = np.append(s_dense, s[-1])

    x_out = cs_x(s_dense)
    y_out = cs_y(s_dense)

    # Heading from spline tangent (CCW+, radians)
    dx = cs_x(s_dense, deriv=1)
    dy = cs_y(s_dense, deriv=1)
    heading_out = np.arctan2(dy, dx)

    # Recompute cumulative distance from actual dense point positions
    cumdist = np.zeros(len(x_out))
    for i in range(1, len(x_out)):
        ddx = x_out[i] - x_out[i - 1]
        ddy = y_out[i] - y_out[i - 1]
        cumdist[i] = cumdist[i - 1] + math.sqrt(ddx * ddx + ddy * ddy)

    return x_out, y_out, heading_out, cumdist


def write_trajectory(path: str, x_out, y_out, heading_out, cumdist):
    with open(path, "w", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(["x", "y", "heading", "cumulative_dist"])
        for x, y, h, d in zip(x_out, y_out, heading_out, cumdist):
            writer.writerow([f"{x:.6f}", f"{y:.6f}", f"{h:.6f}", f"{d:.6f}"])


def main():
    parser = argparse.ArgumentParser(
        description="Cubic-spline resampler: sparse waypoints → dense trajectory.csv"
    )
    parser.add_argument("-i", "--input",   default="teleop_waypoints.csv",
                        help="Input sparse waypoints CSV (default: teleop_waypoints.csv)")
    parser.add_argument("-o", "--output",  default="trajectory.csv",
                        help="Output dense trajectory CSV (default: trajectory.csv)")
    parser.add_argument("-d", "--spacing", type=float, default=0.05,
                        help="Target arc-length spacing in metres (default: 0.05)")
    args = parser.parse_args()

    if args.spacing <= 0:
        print("% ERROR: spacing must be positive")
        sys.exit(1)

    xs, ys = load_waypoints(args.input)
    total_chord = chord_cumulative(xs, ys)[-1]

    print(f"% Input:  {len(xs)} waypoints, total chord length {total_chord:.3f} m  ({args.input})")

    x_out, y_out, heading_out, cumdist = interpolate(xs, ys, args.spacing)

    # Offset so path starts at (0, 0) — keeps the same convention (+x=right, +y=forward)
    x_out -= x_out[0]
    y_out -= y_out[0]

    write_trajectory(args.output, x_out, y_out, heading_out, cumdist)

    actual_spacing = cumdist[-1] / (len(x_out) - 1) if len(x_out) > 1 else 0.0
    print(f"% Output: {len(x_out)} points @ ~{actual_spacing:.4f} m spacing "
          f"→ {args.output}")
    print(f"% Total path length: {cumdist[-1]:.3f} m")
    print(f"% Heading range: {heading_out.min():.3f} to {heading_out.max():.3f} rad")
    print(f"% Done. Load with: python3 mqtt_client_path_follow.py -s")


if __name__ == "__main__":
    main()
