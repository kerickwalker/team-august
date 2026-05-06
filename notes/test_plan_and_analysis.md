# Test Plan and Analysis

## Line following (current focus)

Before tuning or comparing patches:
1. **Calibrate:** `python3 mqtt-client.py --white` (from `mqtt_python`).
2. **Motion check:** `python3 mqtt-client.py --meter`.
3. **Run line follow:** `python3 mqtt-client.py --edge`.
4. Observe: line detection, oscillation, line loss, and `livn` separation between tape and background. Then compare original vs patched behavior.

See `line_following.md` for architecture, patch summary, and workflow.

## Roadmap

1. Define the trial matrix
   - motion primitives: stationary, `-m`, `-p`
   - repetitions
   - speed levels
   - surfaces

2. Standardize run procedure
   - verify or launch `teensy_interface`
   - run the trial
   - stop the trial cleanly
   - archive logs with a trial identifier

3. Add run metadata
   For each run, record:
   - command used
   - date
   - battery or power state
   - environment notes
   - surface notes
   - anything unusual observed

4. Build a log map
   - signal name
   - units
   - expected meaning
   - expected frequency

5. Compute initial `R`
   - use stationary and repeatability runs
   - estimate per-sensor measurement variance

6. Compute initial `Q`
   - use controlled motion runs
   - estimate process-model residual variance

7. Validate and iterate
   - inspect innovations and residuals
   - retune `Q` and `R`

## Motion test pack

### Minimum set
These are the first trials to do.

#### 1. Stationary noise baseline
Purpose:
- estimate measurement noise for `R`

Plan:
- 3 runs
- 60 seconds each
- robot fully still

Command:
```bash
python3 mqtt-client.py -s
```

Note:
- in the current code path, state 200 is stationary and auto-exits after 60 seconds

#### 2. Straight-line repeatability
Purpose:
- assess repeatability and drift in distance and heading during forward motion

Plan:
- 5 runs
- same start pose
- one surface first

Command:
- use `-m`

If no precise measurement tool is available:
- record final endpoint spread visually or with photos

#### 3. Turn repeatability
Purpose:
- assess heading repeatability for turns

Plan:
- 5 runs
- same start pose

Command:
- use `-p`

If no angle tool is available:
- compare final heading against a floor line or wall reference

### Next set when measurement tools are available

#### 4. Absolute distance accuracy
Plan:
- 0.5 m, 1.0 m, and 1.5 m commands
- 5 repetitions each

Goal:
- separate repeatability from absolute calibration bias

#### 5. Absolute turn accuracy
Plan:
- 90 degree and 180 degree turns
- 5 repetitions in each direction

Goal:
- quantify turning bias and spread

### Optional extensions

#### 6. Speed sensitivity
Repeat key distance and turn tests at low, medium, and high speed.

#### 7. Surface sensitivity
Repeat the key trials on at least two surfaces.

## Practical acceptance gates

### IMU stationary behavior
- near-zero mean after bias removal
- stable variance across repeated stationary runs

### Straight repeatability
- endpoint spread should stay small and consistent

### Turn repeatability
- heading spread should stay small and consistent

### Initial bias targets
- distance: start by aiming for mean error within 5%
- turn: start by aiming for within 5 degrees at 90 degrees and within 10 degrees at 180 degrees

### Interpretation rule
- poor repeatability usually means slip, traction, or mechanics
- good repeatability with consistent bias usually means model-parameter mismatch

## Slip diagnosis playbook

### Symptom
Commanded distance or angle varies significantly across identical trials.

### Checks
1. Compare left and right encoder behavior during straight runs.
2. Check whether errors increase at higher speed.
3. Check whether errors are worse on smoother floors.

### Mitigation order
1. Reduce speed and acceleration for calibration runs.
2. Improve tire-floor contact and weight balance.
3. Only then recalibrate wheel radius, wheelbase, or encoder scale.

## MATLAB analysis workflow

MATLAB remains the preferred analysis tool.

For each run, compute:
- mean per IMU channel
- variance per IMU channel
- end-of-run distance error
- end-of-run heading error
- repeatability statistics:
  - mean error
  - standard deviation
  - RMSE

## Kalman-filter design notes

### Recommended state vector
```text
x = [x, y, θ, v, ω]
```

Where:
- `x`, `y`: position in meters
- `θ`: heading in radians
- `v`: linear velocity
- `ω`: angular velocity

### Available measurements
- encoders: wheel-based motion information that can be converted to `v` and `ω`
- gyro: direct angular-rate information
- accelerometer: linear-acceleration information, though noisy
- magnetometer: potential future absolute heading measurement if enabled and validated

### Process model from the notes
```text
x_dot = v * cos(θ)
y_dot = v * sin(θ)
θ_dot = ω
v_dot = (commanded - actual) / tau_v
ω_dot = (commanded - actual) / tau_ω
```

## Covariance estimation strategy

### `R` matrix
Estimate measurement noise from stationary data:
- `R_gyro = var(log_t0_gyro_1.txt)` when still
- `R_acc = var(log_t0_acc_1.txt)` when still
- `R_enc = var(encoder velocity)` when still

### `Q` matrix
Estimate process-noise terms from motion runs:
- execute known commands
- compare measured trajectories or rates against predicted behavior
- estimate
  ```text
  Q = var(measured - predicted)
  ```

### Validation rule
Use innovation statistics:
- innovation = measurement minus predicted measurement
- target behavior is zero-mean innovations with covariance consistent with the model
- if that fails, retune `Q` and `R`

## Next concrete steps
1. Collect 3 stationary datasets of 60 seconds each.
2. Compute an initial `R` matrix in MATLAB.
3. Define the estimator state, process model, and measurement model.
4. Implement an offline Kalman filter using the logged data.
5. Compare estimates against logged pose and other references.
6. Iterate `Q` and `R` based on innovation behavior.
7. Enable and validate magnetometer support later if heading drift remains a problem.
