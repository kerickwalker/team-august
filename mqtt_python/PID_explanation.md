# Line-follow PID on Robobot (`sedge.py`)

This document explains how the line-following controller works, why **Kd** can make the motors sound like they are fighting, what **`lineOutputAlpha`** does, and why **`lineDerivativeBeta`** (CLI `--beta`) was added as a more targeted fix.

## Where the controller runs

- Normalized line samples arrive on MQTT as `T0/livn` roughly every **10 ms** (`sub livn 3` → ~100 Hz).
- `SEdge.decode()` runs `followLine()` **once per sample**, on each new `livn` message.
- Each `followLine()` call sends one `rc <v> <ω>` velocity command to the Teensy.

So **control rate and command rate are both ~100 Hz**, tied to the sensor stream—not to the mission loop’s `t.sleep(0.01)` in `mqtt-final-mission*.py`.

The PID uses a **fixed** sample period `TS_NOMINAL = 0.010` s so **Kp / Kd / Ki stay meaningful** even if MQTT timing jitters slightly.

## Control law (what is computed)

For each sample:

1. **Tracking error** `e = refPosition - lineCenter` (weighted centre of the line under the array; raw `e` is **not** smoothed for P or I).
2. **Derivative term** uses a smoothed error **`e_filt`** only when `lineDerivativeBeta > 0`; see below.
3. **PID output** (before saturation): `u = Kp·e + Ki·integral + Kd·dTerm`.
4. **`u` is clamped** to `[lineYMin, lineYMax]`, then **output smoothing**:  
   `lineY = alpha · raw_y + (1 - alpha) · lineY_prev`  
   where `alpha` is `lineOutputAlpha` (CLI `--alpha`).

The value sent as angular velocity is **`lineY`** (after optional recovery override when the line is invalid).

## Why high Kd causes buzz / fighting wheels

The naive discrete derivative is:

`dTerm ≈ (e[k] − e[k−1]) / Ts`

With **Ts = 0.01 s**, dividing by `Ts` **amplifies** sample-to-sample noise by **100×** before **Kd** multiplies again.

Small jitter in `lineCenterWeighted` (from sensor noise, rounding, or `min(edge_n)` moving the “floor” under the weighted sum) becomes large swings in **`dTerm`**, hence large swings in **`ω`**, **every 10 ms**. The wheel controllers try to track those commands; you hear **audible noise** and it can feel like the motors are fighting.

So the root issue is usually **D acting on a noisy differentiated signal**, not “commands per se,” though sending **100 noisy ω commands per second** certainly excites the drivetrain.

## What `lineOutputAlpha` does

`lineOutputAlpha` is a **first-order low-pass on the commanded turn rate** **after** P, I, and D are combined:

- **Lower α** → smoother `ω`, less high-frequency content → less buzz, but **more lag** on real turning maneuvers.
- It **does not** distinguish noise from real motion: it smooths **everything** leaving the PID.

Many teams use this successfully; it is a blunt but effective tool.

## What `lineDerivativeBeta` does (preferred for Kd noise)

`lineDerivativeBeta` applies an **EWMA (exponential moving average)** **only on the error signal used for the D-path**:

`e_filt = β · e_filt + (1 − β) · e`

Then:

`dTerm = (e_filt − e_filt_prev) / Ts`

- **β = 0** (`lineDerivativeBeta = 0`): each step uses **`e_filt = e`**, so behavior matches the **classic** derivative on raw error (same as before this feature).
- **β > 0**: high-frequency junk in **`e`** is attenuated **before** differencing, so **Kd no longer amplifies single-sample spikes** as aggressively.

**P and I still use raw `e`.** Only **D** sees the filtered error. That lets you **raise Kd** for damping on straights without smoothing away the **proportional** response you need in corners (within the limits of one shared `Kp`).

Typical starting values to try: **β ∈ [0.4, 0.7]** when increasing **Kd** caused motor noise with β = 0.

### Relationship to lowering command rate

**Lowering how often you send `rc`** can reduce mechanical chatter and **also** reduces numerical noise in `(e−e_prev)/Ts` if you **actually slow the PID updates** (larger effective `Ts`). But slowing the whole loop **also adds phase lag to P**, which may hurt cornering if **Kp** is already tuned near the edge.

Filtering **only the D-path** with β targets **derivative noise** without deliberately slowing **P**. If you still hear chatter after tuning β and α, a separate experiment is **sending `rc` at 50 Hz** while computing the PID at 100 Hz (not implemented here)—that addresses motor excitation without changing P’s bandwidth.

## CLI and parameter sets

Mission scripts (`mqtt-final-mission.py`, `mqtt-final-mission-safe.py`) accept:

| Flag       | Maps to             | Role |
|-----------|---------------------|------|
| `--kp`    | `lineKp`            | Proportional gain |
| `--kd`    | `lineKd`            | Derivative gain |
| `--ki`    | `lineKi`            | Integral gain |
| `--alpha` | `lineOutputAlpha`   | Low-pass on **output** ω (0…1) |
| `--beta`  | `lineDerivativeBeta`| EWMA on **error for D only** (0…1) |

Defaults for **`normal`** / **`slow`** live in `SEdge.PARAM_SETS` in `sedge.py`. Overrides patch **`normal`** only, matching the existing `--kp`/`--kd` behavior.

## State resets

- **Line invalid**: integral is cleared; **`e_filt`** is synced to **`e`** and **`lineE_filt_prev`** is cleared so **D does not spike** on garbage samples.
- **`lineControl(..., params=...)`**: integral and **`lineE_filt_prev`** reset when switching named parameter sets.

---

*Tuning tip:* If straights hunt but corners need stiff **Kp**, try **moderate β** with **higher Kd** first; reserve **lower α** for residual output jitter after D is clean.

## Notes from current tuning session

### Current setup reported

```bash
python mqtt-final-mission.py --test --submission line_follow --kp 0.5 --kd 0.4 --alpha 1.0 --beta 0.8
```

Observed behavior:

- motor jitter/chatter is reduced compared to low-beta tests, but still present;
- after some turns, the robot can oscillate on following straights and approach line-loss.

### Interpretation of this behavior

This combination puts the controller in a difficult tradeoff:

- `alpha=1.0` means no output smoothing, so all commanded turn variations are sent directly;
- `beta=0.8` means D is heavily smoothed/delayed;
- `kd=0.4` is relatively strong.

In practice this can produce a D-path that is still energetic enough to excite motor chatter, but delayed enough to damp poorly right after a turn, where transient error dynamics are fastest.

### Practical recommendation from this session

Keep `kp=0.5` for corner authority, then test a less extreme D-filter split:

- reduce `beta` from `0.8` toward `0.5–0.6`;
- reduce `kd` slightly to `0.30–0.35`;
- reintroduce some output smoothing, e.g. `alpha ≈ 0.65`.

Suggested starting point:

```bash
python mqtt-final-mission.py --test --submission line_follow --kp 0.5 --kd 0.32 --alpha 0.65 --beta 0.55
```

Adjustment loop:

- if straight-line sway remains: increase `kd` by small steps (`+0.02..0.03`);
- if motor chatter increases: increase `beta` slightly (`+0.05`) or lower `alpha` slightly (`-0.05`);
- if turn exit feels sluggish: lower `beta` slightly (`-0.05`) or raise `alpha` slightly (`+0.05`).

### Bigger-picture suggestion

If one global `kp` must be high for corners, but straight behavior stays marginal, consider gain scheduling (e.g. gains based on `abs(e)`) so cornering and straight stabilization do not fight each other under a single fixed set.
