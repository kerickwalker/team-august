# ramp_to_bowl_line.py — field tuning backup

Backup of the on-field tuning of `control/tasks/ramp_to_bowl_line.py`. Use
this to restore values if the script is rewritten or refactored.

Date: 2026-04-28
Robot IP: 10.197.219.117

## Phase 1 — Forward off the (short) ramp
| Constant | Value | Note |
|---|---|---|
| F1_DIST_M | 0.78 m | bumped 0.30 → 0.60 → 1.14 → 0.60 → 0.78 (one extra second of travel) |
| F1_SPEED | 0.18 m/s | bumped from 0.08 to make the run snappier |
| ramp_s (drive_for) | 0.5 s | accel/decel cushion so the bowl doesn't slosh |

## Arm/tray servo (locked UP during driving)
| Constant | Value | Note |
|---|---|---|
| SERVO_ARM | 1 | servo id |
| ARM_UP_PWM | -800 | -700 dropped cargo, -900 overheated, -800 is the sweet spot |
| SERVO_SPEED_HOLD | 150 | 300 was overheating the servo; 150 still holds firmly |
| ARM_PRESET_S | 1.5 s | wait for the arm to fully reach UP before driving |

## Phase 2 — Yaw left (face the line we're searching for)
| Constant | Value | Note |
|---|---|---|
| Y1_RAD | 1.57 (90°) | left/CCW |
| Y1_RATE | 0.6 rad/s | |

## Phase 3 — Creep until line detected
| Constant | Value | Note |
|---|---|---|
| F3_SPEED | 0.06 m/s | slow search speed |
| F3_MAX_DIST_M | 0.80 m | safety cap |
| F3_TIMEOUT_S | 15 s | safety cap |
| F3_OVERSHOOT_M | 0.02 m | go 2 cm past the line so the wheels straddle it before turning |
| F3_WARMUP_M | 0.06 m | ignore line detection for the first 6 cm (we end phase 2 already on top of a line) |
| LINE_VALID_THRESHOLD | 500 | livn peak ≥ 500 → line valid |

## Phase 4 — Yaw right (face the bowl)
| Constant | Value | Note |
|---|---|---|
| Y2_RAD | -1.57 (-90°) | right/CW |
| Y2_RATE | 0.6 rad/s | |

## Phase 5 — Gentle approach onto the bowl
| Constant | Value | Note |
|---|---|---|
| F5_DIST_M | 0.40 m | distance to contact the bowl |
| F5_SPEED | 0.05 m/s | very slow — no knocking the bowl over |
| F5_RAMP_S | 0.6 s | extra-soft accel/decel for the cargo |

## Phase 6 — Place bowl on floor (legacy, normally skipped)
| Constant | Value | Note |
|---|---|---|
| ARM_DOWN_PWM | 500 | tray fully lowered |
| ARM_PLACE_SPEED | 120 | smooth descent |
| ARM_PLACE_S | 2.0 s | wait for the arm to settle |
| ARM_RELEASE_PWM | 9999 | \|pos\|>1024 → servo disabled, buzzing stops |

## Phase 7 — Lift arm with cargo (after balls are scooped)
| Constant | Value | Note |
|---|---|---|
| P7_ARM_LIFT_S | 1.5 s | wait for the arm to fully reach UP |

## Phase 8 — Short reverse from the dispenser
| Constant | Value | Note |
|---|---|---|
| F8_DIST_M | 0.25 m | started at 0.50 (too far back), reduced to 0.25 |
| F8_SPEED | 0.10 m/s | gentle reverse |
| ramp_s | 0.4 s | smooth |

## Phase 9 — Yaw right (toward the C-side of the field)
| Constant | Value | Note |
|---|---|---|
| Y9_RAD | -1.57 (-90°) | first attempt was +1.57 (left) — wrong, corrected to right |
| Y9_RATE | 0.6 rad/s | |

## Phase 10 — Short forward (clear the line before the next turn)
| Constant | Value | Note |
|---|---|---|
| F10_DIST_M | 0.80 m | climbed 0.25 → 0.40 → 0.55 → 0.70 → 0.80 in successive trials |
| F10_SPEED | 0.12 m/s | |
| ramp_s | 0.4 s | |

## Phase 11 — Yaw left (line up with the C-bound straight)
| Constant | Value | Note |
|---|---|---|
| Y11_RAD | 2.22 rad (~127°) | climbed 1.57 (90°) → 1.75 → 2.00 → 2.15 → 2.22 |
| Y11_RATE | 0.6 rad/s | |

## Phase 12 — Drive straight to Zone C
| Constant | Value | Note |
|---|---|---|
| F12_DIST_M | 1.60 m | started at 1.20, needed 0.40 m more to reach C |
| F12_SPEED | 0.18 m/s | |
| ramp_s | 0.5 s | |

## Phase 13 — Drop the balls (arm down + release)
| Constant | Value | Note |
|---|---|---|
| ARM_DOWN_PWM | 500 | same as Phase 6 |
| ARM_PLACE_SPEED | 120 | |
| P13_DUMP_S | 1.5 s | wait for balls to fall out |
| ARM_RELEASE_PWM | 9999 | servo release |

## General notes
- All motion is **open-loop time × speed**. Pose feedback in this script is
  not arriving (`pose_seen=False`); likely a topic-prefix mismatch
  (`robobot/kalman/state` vs `robobot/drive/...`) — to be debugged.
- Line detection is only used in Phase 3 (sedge `livn` topic).
- Servo overheating: ARM_UP_PWM=-900 + SERVO_SPEED_HOLD=300 was drawing
  enough torque to warm the winding noticeably. Backed off to -800 + 150.
- To start at a specific phase, use the `--from N` flag.
- Emergency stop: Ctrl+C → script sends motor stop + arm release.

## Total path length
- Phase 1: 0.78 m + Phase 5: 0.40 m → ~1.18 m to bowl
- Phase 8: 0.25 m reverse + Phase 10: 0.80 m + Phase 12: 1.60 m → ~2.65 m total to C
