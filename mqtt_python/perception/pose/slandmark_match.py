#!/usr/bin/env python3
"""
slandmark_match.py
=============================================================================
Match live line-detector output against unique geometric landmarks on the field
map, then produce an absolute (x, y, yaw) fix when a landmark is recognised.

This works similarly to ArUco-based localization. When the camera sees a
distinctive tape pattern, such as a Y-fork, T-junction, or endpoint near a
known fixture, the robot pose can be corrected without relying only on
dead-reckoning.

Inputs per frame
----------------
    line_result    : dict from sline.SLine.process()
                     keys used: line_valid, fork_confirmed, fork_evidence,
                     branches, line_heading, line_curvature

    kalman_prior   : (px, py, yaw)
                     Current Kalman estimate. This is used only as a position
                     gate, so we only consider landmarks close to the prior.
                     The output coordinates still come from the matched
                     landmark's known map position.

Output
------
    dict with the same shape as other svision_pose fix dicts:
        {
            'x':           float,   world X (m)
            'y':           float,   world Y (m)
            'yaw':         float,   world yaw (rad)
            'name':        str,     landmark name
            'kind':        str,     landmark kind
            'confidence':  float,   0..1
            'signature_match': str  short readable reason
        }

    Returns None if no landmark matched.

Supported landmark kinds
------------------------
    fork_y          : confirmed fork with two valid branches and the expected
                      angular separation

    t_intersection  : same detector path as fork_y. The map label tells us
                      whether the pattern is treated as a T or a Y.

    fixture_anchor  : tape endpoint near a known fixture. Currently disabled
                      until a stricter signature is added.

Not yet supported
-----------------
    ramp_start, ramp_end, stair_step

    These need an IMU pitch or vertical acceleration signal in addition to
    camera output. They can be added once IMU history is passed into this
    matcher.
"""
from __future__ import annotations
import math
import time
from typing import Optional, Dict, Any, List, Tuple


# Tunable thresholds. Kept as module-level constants so runners can adjust
# them without changing matcher logic.

ANGLE_TOL_DEG          = 20.0    # allowed branch-angle error from the expected landmark signature
MIN_BRANCH_VALID_COUNT = 2       # both branches must be valid for a fork match
TAPE_END_MAX_M         = 0.80    # if visible tape ends before this distance, treat it as an endpoint
COOLDOWN_S             = 1.0     # do not fire the same landmark again immediately

# Position-gate widening.
# The prior gate prevents landmarks from firing when the Kalman prior is far
# away. At the same time, the prior can drift between absolute fixes. A very
# tight gate may reject a correct landmark just because the prior has drifted.
# This multiplier gives the matcher some tolerance while still relying on
# signature scoring to reject wrong matches.
MIN_PRIOR_GATE_RATIO   = 3.0

# Near-miss logging.
# If the prior is close to a landmark but the matcher does not fire, log the
# reason occasionally. This makes it easier to tell whether the problem is
# localization drift, perception tuning, or an unsupported signature.
NEAR_MISS_LOG_RATIO     = 2.5
_NEAR_MISS_LOG_PERIOD_S = 2.0


def _wrap_deg(a: float) -> float:
    return (a + 180.0) % 360.0 - 180.0


class SLandmarkMatcher:
    ready = False

    def __init__(self):
        self._field = None
        self._landmarks: List = []
        self._last_fired: Dict[str, float] = {}          # landmark name -> timestamp
        self._last_near_miss_log: Dict[str, float] = {}  # landmark name -> timestamp
        self.match_count: int = 0
        self.last: Optional[Dict[str, Any]] = None

    # -------------------------------------------------------------------
    def setup(self) -> bool:
        """Load FIELD.landmarks and mark the matcher as ready."""
        try:
            from field_map.field_map_2026 import FIELD
        except Exception as exc:
            print(f"% SLandmarkMatcher: field map load failed: {exc}")
            return False

        self._field = FIELD
        self._landmarks = list(FIELD.landmarks)
        self.ready = bool(self._landmarks)
        print(f"% SLandmarkMatcher: ready  landmarks={len(self._landmarks)}  "
              f"kinds={sorted({lm.kind for lm in self._landmarks})}")
        return self.ready

    # -------------------------------------------------------------------
    def process(self,
                line_result: Optional[Dict[str, Any]],
                kalman_prior: Tuple[float, float, float],
                t_now: Optional[float] = None
                ) -> Optional[Dict[str, Any]]:
        """Match the current line detection against known landmarks.

        Returns a fix dict when the match is confident enough.
        Returns None when no landmark matches this frame.
        """
        if not self.ready or line_result is None:
            return None

        px, py, yaw = kalman_prior
        t_now = t_now if t_now is not None else time.time()

        best: Optional[Dict[str, Any]] = None
        best_score = 0.0

        for lm in self._landmarks:
            # Avoid sending repeated fixes for the same landmark too quickly.
            t_last = self._last_fired.get(lm.name, 0.0)
            if t_now - t_last < COOLDOWN_S:
                continue

            # Position gate: only consider landmarks close to the Kalman prior.
            d_prior = math.hypot(px - lm.position.x, py - lm.position.y)
            gate_radius = lm.confidence_radius * MIN_PRIOR_GATE_RATIO
            if d_prior > gate_radius:
                # Log near misses when the prior is close enough that a match
                # might have been expected.
                if d_prior < gate_radius * NEAR_MISS_LOG_RATIO:
                    last_log = self._last_near_miss_log.get(lm.name, 0.0)
                    if t_now - last_log > _NEAR_MISS_LOG_PERIOD_S:
                        print(f"% landmark gate-miss: {lm.name} ({lm.kind}) "
                              f"prior_dist={d_prior:.2f}m > gate={gate_radius:.2f}m "
                              f"(prior drift is too large; an ArUco fix may be needed)")
                        self._last_near_miss_log[lm.name] = t_now
                continue

            score, why = self._score(lm, line_result, kalman_prior)
            if score <= 0.0:
                # The prior is inside the gate, but the observed signature did
                # not match. This usually points to perception-side tuning, for
                # example fork evidence not reaching the confirmation threshold.
                last_log = self._last_near_miss_log.get(lm.name, 0.0)
                if t_now - last_log > _NEAR_MISS_LOG_PERIOD_S:
                    print(f"% landmark sig-miss: {lm.name} ({lm.kind}) "
                          f"prior_dist={d_prior:.2f}m gate={gate_radius:.2f}m  "
                          f"reason: {why}")
                    self._last_near_miss_log[lm.name] = t_now
                continue

            # Combine signature quality with how well-centred the prior is
            # inside the landmark gate.
            centring = max(0.0, 1.0 - d_prior / gate_radius)
            confidence = 0.5 * score + 0.5 * centring

            if confidence > best_score:
                best_score = confidence
                best = {
                    'x':                lm.position.x,
                    'y':                lm.position.y,
                    'yaw':              math.radians(lm.expected_yaw_deg)
                                         if lm.expected_yaw_deg is not None else yaw,
                    'name':             lm.name,
                    'kind':             lm.kind,
                    'confidence':       round(confidence, 3),
                    'signature_match':  why,
                }

        if best is not None:
            self._last_fired[best['name']] = t_now
            self.match_count += 1
            self.last = dict(best)

        return best

    # -------------------------------------------------------------------
    # Kind-specific scoring functions. Each returns (score in [0..1], reason).
    # -------------------------------------------------------------------
    def _score(self, lm, line_result, kalman_prior) -> Tuple[float, str]:
        if lm.kind in ("fork_y", "t_intersection"):
            return self._score_fork(lm, line_result)
        if lm.kind == "fixture_anchor":
            # Disabled for now because the current endpoint signature can fire
            # on unrelated tape ends. Re-enable after adding a stricter check.
            return 0.0, "fixture_anchor disabled to avoid false positives"
        return 0.0, f"kind '{lm.kind}' is not yet supported by the camera-only matcher"

    @staticmethod
    def _score_fork(lm, line_result) -> Tuple[float, str]:
        if not line_result.get("fork_confirmed"):
            return 0.0, "no fork confirmed"

        branches = line_result.get("branches") or []
        valid = [b for b in branches if b.get("valid")]
        if len(valid) < MIN_BRANCH_VALID_COUNT:
            return 0.0, "fewer than 2 valid branches"

        # Angular separation between the two detected branches.
        h0 = math.degrees(valid[0].get("heading", 0.0))
        h1 = math.degrees(valid[1].get("heading", 0.0))
        observed_sep = abs(_wrap_deg(h0 - h1))

        expected_sep = float(lm.expected_signature.get("fork_angle_deg", 90.0))
        delta = abs(observed_sep - expected_sep)
        if delta > ANGLE_TOL_DEG:
            return 0.0, f"angle delta {delta:.1f}° > tol {ANGLE_TOL_DEG}°"

        score = max(0.0, 1.0 - delta / ANGLE_TOL_DEG)
        return score, f"fork angle obs={observed_sep:.1f}° exp={expected_sep:.0f}° delta={delta:.1f}°"

    @staticmethod
    def _score_fixture_anchor(lm, line_result) -> Tuple[float, str]:
        if not line_result.get("line_valid"):
            return 0.0, "no valid line"

        # Endpoint check: line is visible, but the active branch ends nearby.
        # Points are in robot frame, where x is forward distance.
        active_branch = (line_result.get("branches") or [None])[0]
        if not active_branch:
            return 0.0, "no active branch"
        pts = active_branch.get("points") or []
        if not pts:
            return 0.0, "active branch has no points"

        farthest = max(p[0] for p in pts)
        if farthest > TAPE_END_MAX_M:
            return 0.0, f"tape continues past {TAPE_END_MAX_M:.2f} m (saw {farthest:.2f})"

        score = 1.0 - farthest / TAPE_END_MAX_M
        return score, f"tape endpoint at {farthest:.2f} m"

    # -------------------------------------------------------------------
    def stats(self) -> Dict[str, Any]:
        return {
            "ready":       self.ready,
            "landmarks":   len(self._landmarks),
            "matches":     self.match_count,
            "last_fired":  dict(self._last_fired),
            "last":        dict(self.last) if self.last else None,
        }


landmark_matcher = SLandmarkMatcher()