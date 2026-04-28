"""Tape detection thresholds for sline.SLine.

Auto-generated or manually edited. Loaded by sline.py during import.

You can tune these values live with:
    python test_sline_tuner.py --host <robot_ip>
then press 'S' to save the updated values back into this file.

Group A - colour mask (very sensitive to lighting, usually needs on-site tuning):
    WHITE_V_MIN, WHITE_S_MAX

Group B - ROI / strip-based search:
    LINE_ROI_TOP_FRAC, N_STRIPS, MIN_BLOB_AREA,
    INIT_WINDOW_PX, TRACK_WINDOW_PX, TAPE_VERIFY_MARGIN_PX

Group C - fork (Y/T) detection:
    FORK_SEARCH_STRIPS, FORK_MIN_SEP_PX, FORK_MIN_GROUND_SEP_M,
    FORK_CANDIDATE_MIN, FORK_CONFIRMED_MIN

Group D - line validity:
    MIN_VALID_STRIPS
"""

# --- A: colour mask ---------------------------------------------------------
WHITE_V_MIN            = 160        # minimum HSV V value for detecting white tape (0..255)
WHITE_S_MAX            = 60         # maximum HSV S value for detecting white (0..255)

# --- B: ROI / strip search --------------------------------------------------
LINE_ROI_TOP_FRAC      = 0.45       # top edge of ROI as a fraction of image height
N_STRIPS               = 8          # number of horizontal strips to scan
MIN_BLOB_AREA          = 150        # ignore blobs smaller than this area (in pixels)
INIT_WINDOW_PX         = 220        # search window size for initial centroid detection (px)
TRACK_WINDOW_PX        = 160        # smaller window used once tracking is stable (px)
TAPE_VERIFY_MARGIN_PX  = 5          # half-width margin used to confirm tape thickness

# --- C: fork (Y / T) detection ----------------------------------------------
FORK_SEARCH_STRIPS     = 3          # number of top strips checked for a second branch
FORK_MIN_SEP_PX        = 30         # minimum separation between branches (pixels)
FORK_MIN_GROUND_SEP_M  = 0.05       # minimum separation projected onto the ground (metres)
FORK_CANDIDATE_MIN     = 2          # strips with a second branch needed to mark a "candidate"
FORK_CONFIRMED_MIN     = 5          # strips with a second branch needed to confirm a fork

# --- D: line validity -------------------------------------------------------
MIN_VALID_STRIPS       = 3          # if fewer strips are valid, line_valid is set to False