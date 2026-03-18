#!/bin/bash

# Quick Start: Robot Teleoperation with Kalman Filter

echo "╔════════════════════════════════════════════════════════╗"
echo "║  Robot Teleoperation with Kalman Filter - Quick Start  ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""

cd mqtt_python 2>/dev/null || { echo "Error: Must run from team-august directory"; exit 1; }

# Check if MQTT packages are installed
echo "[1/3] Checking dependencies..."
python3 -c "import paho.mqtt.client" 2>/dev/null || {
    echo "  ✗ paho-mqtt not installed"
    echo "  Install with: pip install paho-mqtt"
    exit 1
}
echo "  ✓ paho-mqtt installed"

# Verify required files exist
echo ""
echo "[2/3] Checking required files..."
for file in mqtt-client.py teleop_input.py kalman_output.py skalman.py uservice.py; do
    if [ -f "$file" ]; then
        echo "  ✓ $file"
    else
        echo "  ✗ $file MISSING"
    fi
done

# Check Python syntax
echo ""
echo "[3/3] Verifying Python syntax..."
for file in mqtt-client.py teleop_input.py kalman_output.py; do
    if python3 -m py_compile "$file" 2>/dev/null; then
        echo "  ✓ $file syntax OK"
    else
        echo "  ✗ $file has syntax errors"
    fi
done

echo ""
echo "╔════════════════════════════════════════════════════════╗"
echo "║              SETUP COMPLETE - Ready to Go!             ║"
echo "╚════════════════════════════════════════════════════════╝"
echo ""
echo "Usage Instructions (3 Terminals):"
echo ""
echo "Terminal 1: Start MQTT Broker (if not running)"
echo "  $ mosquitto"
echo ""
echo "Terminal 2: Start Robot Control"
echo "  $ cd mqtt_python"
echo "  $ python3 mqtt-client.py -i localhost"
echo ""
echo "Terminal 3: Send Teleoperation Commands"
echo "  $ cd mqtt_python"
echo "  $ python3 teleop_input.py -i localhost"
echo ""
echo "Terminal 4 (Optional): Monitor Kalman State"
echo "  $ cd mqtt_python"
echo "  $ python3 kalman_output.py -i localhost"
echo ""
echo "Example velocity commands to try:"
echo "  0.5 0.0    -> Forward"
echo "  -0.5 0.0   -> Backward"
echo "  0.0 0.5    -> Turn left"
echo "  0.0 -0.5   -> Turn right"
echo "  0.3 0.3    -> Forward-left"
echo "  0.0 0.0    -> Stop"
echo ""
echo "For more details, see: TELEOPERATION_README.md"
echo ""
