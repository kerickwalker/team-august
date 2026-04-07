# Robot Notes Overview

This bundle reorganizes `notes.md` into separate reference files without repeating the same explanations in multiple places.

## Current date in notes
- 2026-04-07

## Main objective
- Collect trial data for sensor calibration and validation.
- Use the results to support Kalman-filter-based sensor fusion.
- Estimate and tune covariance matrices `Q` (process noise) and `R` (measurement noise).

## Immediate next-session priority
1. **Path following (current focus):** Pure Pursuit controller implemented. Next step: generate a `trajectory.csv`, verify Kalman filter is publishing on `robobot/kalman/state`, then run `--usestate 105`. See `path_following.md`.
2. **Magnetometer (deferred):** AK8963 unreachable on I2C (0x0C). Fix: enable BYPASS_EN in INT_PIN_CFG (0x37). See `change_log.md` and `sensor_reference.md`.
3. Use existing stationary dataset as baseline for Kalman fusion work; after mag fix, collect new stationary data with magnetometer.

## What each file is for

### `environment_and_logging.md`
Use this for paths, log locations, log folders, logging semantics, timestamp handling, and the full log-file inventory.

### `commands_and_workflow.md`
Use this for the copy-paste command sheet, startup and shutdown sequence, MQTT log toggling, option flags, and practical operating constraints.

### `test_plan_and_analysis.md`
Use this for the trial roadmap, motion test pack, acceptance criteria, slip diagnosis, MATLAB analysis workflow, and Kalman-filter setup notes.

### `change_log.md`
Use this for the code modifications already made, why they were made, and what has been verified after those changes.

### `system_architecture.md`
Use this for the end-to-end system structure: firmware, interface, MQTT layer, Python client, hardware summary, and component responsibilities.

### `sensor_reference.md`
Use this for how each sensor-derived signal is produced, what the main signals mean, what is currently disabled, and the implications for estimation.

### `line_following.md`
Use this for the line-following stack: active controller in Python, Teensy vs interface vs Python roles, first Python patch (weighted center, center-follow, dropout recovery), and calibration/test workflow (`--white`, `--meter`, `--edge`).

### `path_following.md`
Use this for the Pure Pursuit trajectory-following controller: algorithm, file structure, CSV format, tuning guide, edge cases, and how to swap to a Stanley controller.

### `llm_operator_rules.md`
Use this for the Copilot/LLM-specific rules and the preferred way of working in future sessions.

## Key context
- The primary logs of interest are in `robobot/teensy_interface/build`.
- There is also a top-level system log tree in `/home/local/svn/log`.
- `mqtt-client.py` depends on `teensy_interface` being running; the interface is not just a logger.
- Only one active MQTT master is allowed at a time.
- Edge sensor logging behavior in `log_t0_edge_liv.txt` and `log_t0_edge_livn.txt` still deserves follow-up.

## Pending inputs
- Additional wiki links or external documentation can still be added later and merged into this bundle.
