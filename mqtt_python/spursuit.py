import numpy as np
import csv

LOOKAHEAD_DIST = 0.5   # metres — distance ahead on trajectory to aim for
MAX_LINVEL     = 0.3   # m/s   — constant forward speed during tracking
MAX_TURNRATE   = 1.0   # rad/s — saturation limit for angular velocity output
SEARCH_WINDOW  = 30    # max points to scan forward when finding nearest point
                       # (30 points × 0.05 m spacing = 1.5 m window)


class SPurePursuit:
    """
    Pure Pursuit path-following algorithm.
    No MQTT, no robot-specific imports — only numpy and the trajectory data.

    Usage:
        pursuit.load_trajectory("traj.csv")   # once, before running
        pursuit.reset()                        # call before each new run
        linvel, turnrate = pursuit.compute_command(x, y, heading, speed)
        if pursuit.at_end():
            stop_robot()
    """

    def __init__(self):
        self._traj = None       # np.ndarray shape (N, 4): x, y, heading, cumulative_dist
        self._nearest_idx = 0   # stateful: last known nearest index (avoids O(N) search)

    def load_trajectory(self, csv_path: str) -> bool:
        """
        Load trajectory from CSV file.  Expected columns (in any order):
            x, y, heading, cumulative_dist
        Returns True on success, False on any error.
        """
        try:
            rows = []
            with open(csv_path, newline='') as f:
                reader = csv.DictReader(f)
                for row in reader:
                    rows.append([
                        float(row['x']),
                        float(row['y']),
                        float(row['heading']),
                        float(row['cumulative_dist']),
                    ])
            if len(rows) < 2:
                print(f"% SPurePursuit: trajectory too short ({len(rows)} points) in '{csv_path}'")
                self._traj = None
                return False
            self._traj = np.array(rows, dtype=float)
            self._nearest_idx = 0
            print(f"% SPurePursuit: loaded {len(rows)} waypoints from '{csv_path}'"
                  f" (total dist {self._traj[-1, 3]:.2f} m)")
            return True
        except Exception as e:
            print(f"% SPurePursuit: failed to load '{csv_path}': {e}")
            self._traj = None
            return False

    def reset(self):
        """Reset nearest-point index to 0.  Call before starting a new run."""
        self._nearest_idx = 0

    def is_loaded(self) -> bool:
        return self._traj is not None

    def at_end(self) -> bool:
        """True once the nearest point has reached the last trajectory point."""
        if self._traj is None:
            return True
        return self._nearest_idx >= len(self._traj) - 1

    def compute_command(self, x: float, y: float, heading: float, speed: float, linvel: float = None):
        """
        Compute (linvel, turnrate) for one control step.

        Returns (0.0, 0.0) when:
          - no trajectory is loaded
          - the end of the trajectory has been reached
          - the lookahead point collapses onto the robot (numerical guard)

        Algorithm
        ---------
        1. Forward search from _nearest_idx (bounded by SEARCH_WINDOW) for the
           closest trajectory point; advance _nearest_idx.
        2. Walk forward from _nearest_idx until cumulative_dist exceeds
           nearest_dist + LOOKAHEAD_DIST → lookahead point (x_l, y_l).
           If the trajectory ends before that distance, use the final point.
        3. Transform (x_l, y_l) into the robot frame using the robot heading.
        4. Curvature κ = 2·dy_robot / (dx_robot² + dy_robot²)  (Pure Pursuit formula)
        5. turnrate = clamp(κ · MAX_LINVEL,  ±MAX_TURNRATE)
           linvel   = MAX_LINVEL  (constant-speed tracking)
        """
        if self._traj is None or self.at_end():
            return (0.0, 0.0)

        traj = self._traj
        N = len(traj)

        # --- Step 1: nearest point (bounded forward search) ---
        # Trajectory y uses +y=right convention; robot y uses +y=left convention.
        # Negate trajectory y so both are in the same frame for distance computation.
        end_search = min(self._nearest_idx + SEARCH_WINDOW, N)
        seg_x =  traj[self._nearest_idx:end_search, 0] - x
        seg_y = -traj[self._nearest_idx:end_search, 1] - y
        local_idx = int(np.argmin(seg_x * seg_x + seg_y * seg_y))
        self._nearest_idx += local_idx

        if self.at_end():
            return (0.0, 0.0)

        # --- Step 2: lookahead point ---
        target_cum_dist = traj[self._nearest_idx, 3] + LOOKAHEAD_DIST
        lookahead_idx = self._nearest_idx
        while lookahead_idx < N - 1 and traj[lookahead_idx, 3] < target_cum_dist:
            lookahead_idx += 1
        x_l =  traj[lookahead_idx, 0]
        y_l = -traj[lookahead_idx, 1]   # negate: trajectory +y=right → robot frame +y=left

        # --- Step 3: transform to robot frame ---
        dx_w = x_l - x
        dy_w = y_l - y
        ch = np.cos(heading)
        sh = np.sin(heading)
        dx_r =  ch * dx_w + sh * dy_w   # forward in robot frame
        dy_r = -sh * dx_w + ch * dy_w   # left in robot frame

        # --- Step 4: curvature ---
        l_sq = dx_r * dx_r + dy_r * dy_r
        if l_sq < 1e-6:
            return (MAX_LINVEL, 0.0)
        kappa = 2.0 * dy_r / l_sq

        # --- Step 5: output ---
        if linvel is None:
            linvel = float(MAX_LINVEL)
        turnrate = float(np.clip(kappa * linvel, -MAX_TURNRATE, MAX_TURNRATE))
        linvel   = float(linvel)
        return (linvel, turnrate)


# module-level singleton — import with: from spursuit import pursuit
pursuit = SPurePursuit()
