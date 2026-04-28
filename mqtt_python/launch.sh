#!/bin/bash
# Usage:
#   ./launch.sh [robot-ip] [mode]
#
# Modes:
#   full     default; core + perception + kalman_output + teleop + web map
#   drive    core + perception + kalman_output + teleop
#   watch    core + perception + kalman_output + web map
#   minimal  core + perception + kalman_output
#
# Examples:
#   ./launch.sh
#   ./launch.sh 10.197.219.117
#   ./launch.sh 10.197.219.117 drive
#
# All processes open as tabs in the same Terminal.app window.
# Close any tab with Cmd+W when you are done with it.

DIR="$(cd "$(dirname "$0")" && pwd)"
IP="${1:-10.197.219.117}"
MODE="${2:-full}"
PY="/Users/elifpulukcu/Desktop/team-august/venv/bin/python3"

# Make project imports work no matter which subdirectory starts the script.
export PYTHONPATH="$DIR"

open_tab() {
    local label="$1"
    local cmd="$2"

    # Bring Terminal to the front first, otherwise Cmd+T may go to the app
    # currently in focus instead of opening a new Terminal tab.
    osascript -e "
        tell application \"Terminal\"
            activate
            tell application \"System Events\" to keystroke \"t\" using {command down}
            delay 0.7
            do script \"cd '$DIR' && export PYTHONPATH='$DIR' && $cmd\" in front window
            set custom title of front tab of front window to \"$label\"
        end tell"
}

# Tab 1: robot control core. Motors live here; without this, the rest is read-only.
osascript -e "
    tell application \"Terminal\"
        activate
        do script \"cd '$DIR' && export PYTHONPATH='$DIR' && $PY runners/mqtt_client_core.py\" in front window
        set custom title of front tab of front window to \"1 · core\"
    end tell"

sleep 0.5

# Tab 2: perception pipeline.
# Runs line detection, ArUco detection, landmark matching, vision pose output,
# and CSV logging. The annotated overlay is also published as MJPEG on port 7124.
open_tab "2 · perception" "$PY runners/live_perception_overlay.py --host $IP --params calibration/camera_params.npz --print-terminal --mjpeg-port 7124 --no-window"

sleep 0.5

# Tab 3: Kalman state monitor in the terminal.
open_tab "3 · kalman" "$PY sensors/fusion/kalman_output.py -i $IP"

case "$MODE" in
    full|drive)
        sleep 0.5
        # Tab 4: manual teleop. Type velocity commands here; 0.0 0.0 stops the robot.
        open_tab "4 · teleop" "$PY control/teleop/teleop_input.py -i $IP"
        ;;
esac

case "$MODE" in
    full|watch)
        sleep 0.5
        # Tab 5: web dashboard with field map, live pose, vision fixes, and camera streams.
        open_tab "5 · web" "$PY -m web_app.field_web_app --broker $IP --camera-stream http://$IP:7123/stream.mjpg --annotated-stream http://localhost:7124/stream.mjpg"
        ;;
esac

echo ""
echo "════════════════════════════════════════════════════════════════"
echo "  Launched with robot IP: $IP  (mode: $MODE)"

case "$MODE" in
    full)    echo "  Tabs: core | perception | kalman_output | teleop | web_map" ;;
    drive)   echo "  Tabs: core | perception | kalman_output | teleop" ;;
    watch)   echo "  Tabs: core | perception | kalman_output | web_map" ;;
    minimal) echo "  Tabs: core | perception | kalman_output" ;;
esac

case "$MODE" in
    full|watch)
        echo ""
        echo "  WEB DASHBOARD:"
        echo ""
        echo "      http://localhost:8050"
        echo ""
        echo "  Camera streams are available in the CAMERA panel."
        ;;
esac

echo "════════════════════════════════════════════════════════════════"