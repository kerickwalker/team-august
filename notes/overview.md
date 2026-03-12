# Robot Notes Overview

This bundle reorganizes `notes.md` into separate reference files without repeating the same explanations in multiple places.

## Current date in notes
- 2026-03-12

## Main objective
- Collect trial data for sensor calibration and validation.
- Use the results to support Kalman-filter-based sensor fusion.
- Estimate and tune covariance matrices `Q` (process noise) and `R` (measurement noise).

## Immediate next-session priority
1. **Line following (current focus):** Calibrate, run existing system, then test the Python patch. Order: `--white` → `--meter` → `--edge`. See `line_following.md` for architecture, patch summary, and workflow. Magnetometer is deferred until line following is in a good state.
2. **Magnetometer (after line following):** Firmware/interface/MQTT path verified; failure is at I2C (AK8963 unreachable). Next step: enable I2C bypass (INT_PIN_CFG 0x37, BYPASS_EN) and re-test; see `change_log.md` and `sensor_reference.md` (Magnetometer).
3. Use the existing stationary dataset without magnetometer as the baseline for later fusion work; after mag fix, collect new stationary data with magnetometer and compare.

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
