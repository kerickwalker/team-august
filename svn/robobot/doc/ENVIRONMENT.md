# Robobot Environment Baseline

This file defines the baseline environment needed to reproduce the clean branch.

## Platform
- OS: Linux (Raspberry Pi/Ubuntu-class)
- Shell: bash
- Working tree expected at: `/home/local/svn/robobot`

## Required Tools
- `python3` (recommended: 3.10+)
- `pip3`
- `mosquitto-clients` (`mosquitto_pub`)
- `git`
- `g++`
- `make`
- `cmake`

## Runtime Entry Points
- Teensy interface: `teensy_interface/build/teensy_interface`
- MQTT client: `mqtt_python/mqtt-client.py`
- Camera streamer: `stream_server/stream_server.py`
- Startup helper: `setup/on_reboot.bash`

## Rebuild / Verify Flow
1. Run `setup/bootstrap.sh` to install baseline packages.
2. Run `setup/verify_env.sh` to verify tool availability and versions.
3. Rebuild native components in each `build/` folder as needed.
4. Run a smoke test:
   - Start `teensy_interface`
   - Start `mqtt-client.py`
   - Send one MQTT command and verify response/log activity.

## Notes
- Build folders and generated binaries are intentionally ignored by Git.
- If exact binary reproducibility is needed, pin compiler/toolchain versions here.
- Tag known-good states in Git, for example: `clean-v1`, `clean-v1.1`.
