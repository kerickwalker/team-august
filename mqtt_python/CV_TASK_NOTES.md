# Computer Vision Task Notes (Gate + Line)

This file is a handoff snapshot of the current CV work so it can be resumed quickly.

## 1) Gate Detection (`sgate.py`, used by `computer_vision.py`)

### Current behavior
- Detects orange/yellow uprights using HSV mask:
  - `hsv_lower = [3, 120, 80]`
  - `hsv_upper = [30, 255, 255]`
- Builds bar rectangles from contours.
- Merges vertically stacked bar segments before filtering:
  - `merge_x_tol_px = 18`
  - `merge_y_gap_px = 35`
- Bar filter:
  - `min_height_px = 100`
- Gate pair rules:
  - width between bar centers must satisfy:
    - `min_gate_width_px = 100`
    - `max_gate_width_px = 600`
  - geometric rule:
    - each bar height must be at least pair width (`bar_h >= pair_w`)
    - unless that bar is clipped by top/bottom image edge
- Supports multiple gate candidates; one primary gate is selected:
  - candidate center closest to image center

### Debug/overlay features currently added
- Rejected bars (red) with reason text.
- Accepted bars (blue).
- Rejected pairs (red lines) with reason:
  - `w<100`, `w>600`, `left_h<w`, `right_h<w`
- Valid pair candidates (orange).
- Selected primary gate (green).
- Per-frame reason counters in overlay:
  - `bars reject: ...`
  - `pairs reject: ...`

### Main problems observed
- In cluttered scenes with many yellow structures, still unstable.
- Even with merge and reason overlays, some obvious gates are intermittently missed.
- Gate selection can still latch onto unintended structures depending on frame composition.

### Good next steps
- Add temporal tracking (track selected bars across frames with ID + hysteresis).
- Penalize candidates with large bar-height mismatch.
- Add stricter pair symmetry constraints:
  - similar bar heights, similar y-level, similar width.
- Add optional ROI mask presets per task (disabled currently by request, but likely useful later).

---

## 2) Line Detection (`svline.py`, used by `line_vision.py`)

### Current behavior
- Uses HSV-based white-ish mask (NOT grayscale, NOT edge detector).
- ROI is configurable and currently defaults to:
  - top half (`roi_mode = top`, `roi_fraction = 0.50`) in `line_vision.py`
- White/ground color model (derived from sampled run images):
  - line-like:
    - `line_white_min_v = 166`
    - `line_white_max_s = 32`
  - ground suppression:
    - `ground_dark_max_v = 92`
    - `ground_sat_min_s = 18`
  - combined mask:
    - line mask AND NOT ground mask
- Morphological cleanup:
  - close kernel `(13,3)`, open kernel `(3,3)`
- Single-line strategy:
  - evaluate contours in ROI
  - reject weak/short-span/non-horizontal candidates
  - choose one best contour by score (size + center preference)
- Extra constraints:
  - `min_line_span_frac = 0.20`
  - `min_horizontal_aspect = 2.8`
  - in top mode, reject contours touching ROI bottom edge (`avoid_bottom_edge_px = 8`)

### Debug/overlay features currently added
- Shows ROI boundaries and mode.
- Draws selected contour (yellow) that detector considers the line.
- Shows line offset + confidence counter.

### Main problems observed
- Still sometimes picks large connected white markings not representing the desired track line.
- Highly sensitive to lighting and white objects in background.
- Single contour may represent merged unrelated markings.

### Good next steps
- Add skeletonization / centerline extraction from mask (instead of largest contour shape).
- Fit line/curve model and reject contours that fail model smoothness.
- Add temporal filtering of line offset (EMA/Kalman).
- Add optional lane-prior region (geometric prior) for where line is expected in image.
- Consider alternative color spaces or adaptive thresholding by local brightness.

---

## 3) Runners and default commands

### Gate runner
- File: `mqtt_python/computer_vision.py`
- Example:
  - `python3 mqtt_python/computer_vision.py --live-stream --video-out`

### Line runner
- File: `mqtt_python/line_vision.py`
- Defaults:
  - live stream port: `5001`
  - max-time: `20s`
  - top ROI, 50%
- Example:
  - `python3 mqtt_python/line_vision.py --live-stream --video-out`

---

## 4) Suggested restart plan next session

1. Re-test line runner with current defaults and capture 2-3 representative failure frames.
2. For those frames, log:
   - selected contour area, bbox, aspect, score
   - top 3 rejected candidate reasons
3. Implement one stronger geometric model for line (centerline/curve fit).
4. Only after line reliability is acceptable, revisit gate tracking stabilization.

