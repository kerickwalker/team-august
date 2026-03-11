#!/usr/bin/env bash
set -euo pipefail

if [[ "${EUID}" -ne 0 ]]; then
  SUDO="sudo"
else
  SUDO=""
fi

if ! command -v apt-get >/dev/null 2>&1; then
  echo "apt-get not found. Install dependencies manually for your distro."
  exit 1
fi

${SUDO} apt-get update
${SUDO} apt-get install -y \
  python3 \
  python3-pip \
  mosquitto-clients \
  git \
  g++ \
  make \
  cmake

echo "Bootstrap complete. Run setup/verify_env.sh next."
