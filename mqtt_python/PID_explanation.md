# Line-Follow PID On Robobot (`sedge.py`)

This note describes the current line-follow controller and the tuning flags used
from `mqtt-final-mission.py`.

## Timing

Normalized line samples arrive as `T0/livn` about every 10 ms. The controller
uses a fixed sample time:

```python
TS_NOMINAL = 0.010
```

`edge_nInterval` is still measured for diagnostics, but it does not rescale the
PID math. If the measured period drifts far from 10 ms, `sedge.py` prints a
warning.

## Control Law

For each valid line sample:

```python
e = refPosition - lineCenter
derivative_raw = (e - previous_error) / TS_NOMINAL
dTerm = alpha * previous_dTerm + (1.0 - alpha) * derivative_raw
u = Kp * e + Ki * integral + Kd * dTerm
lineY = clamp(u, lineYMin, lineYMax)
```

`lineY` is sent directly as the turn rate. There is no output smoothing on
`lineY`.

## Alpha

`--alpha` maps to `lineDerivativeAlpha` and smooths only the derivative term:

- `alpha = 0.0`: raw derivative, no smoothing.
- `alpha = 0.7`: `dTerm = 0.7 * previous_dTerm + 0.3 * derivative_raw`.
- `alpha = 1.0`: hold the previous derivative value.

P and I still use the raw error. This means alpha can reduce derivative noise
without deliberately delaying the proportional steering response.

`--beta` is kept only as a deprecated alias for `--alpha`.

## CLI Flags

| Flag | Maps to | Role |
|---|---|---|
| `--kp` | `lineKp` | Proportional gain |
| `--kd` | `lineKd` | Derivative gain |
| `--ki` | `lineKi` | Integral gain |
| `--alpha` | `lineDerivativeAlpha` | Derivative smoothing |
| `--beta` | `lineDerivativeAlpha` | Deprecated alias for `--alpha` |

Overrides are applied to both `normal` and `slow` parameter sets.

## Resets

When the line is invalid, the integral, previous error, and previous derivative
are reset. When `lineControl(..., params=...)` switches parameter sets, those
same state variables are reset so the derivative does not spike across modes.
