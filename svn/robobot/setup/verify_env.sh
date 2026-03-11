#!/usr/bin/env bash
set -euo pipefail

missing=0

check_cmd() {
  local cmd="$1"
  local label="$2"
  if command -v "$cmd" >/dev/null 2>&1; then
    echo "[OK] ${label}: $(command -v "$cmd")"
  else
    echo "[MISSING] ${label} (${cmd})"
    missing=1
  fi
}

echo "Checking required commands..."
check_cmd python3 "Python 3"
check_cmd pip3 "pip3"
check_cmd mosquitto_pub "Mosquitto client"
check_cmd git "Git"
check_cmd g++ "GNU C++"
check_cmd make "make"
check_cmd cmake "CMake"

echo
echo "Version snapshot:"
python3 --version || true
pip3 --version || true
mosquitto_pub -h 2>/dev/null | head -n 1 || true
git --version || true
g++ --version | head -n 1 || true
make --version | head -n 1 || true
cmake --version | head -n 1 || true

if [[ "$missing" -ne 0 ]]; then
  echo
  echo "Environment check failed: one or more required tools are missing."
  exit 2
fi

echo
echo "Environment check passed."
