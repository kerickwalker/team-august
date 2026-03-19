"""Shared helpers for starting/stopping teensy_interface as a managed subprocess."""
import os
import signal
import subprocess
import time as t

TEENSY_INTERFACE_BIN = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "../teensy_interface/build/teensy_interface"
)

_ti_proc = None

def start_teensy_interface():
  """Kill any existing teensy_interface, start a fresh one, and wait for it to connect."""
  global _ti_proc
  try:
    subprocess.run(["pkill", "-x", "teensy_interfac"], check=False)  # 15-char kernel comm limit
    t.sleep(1.5)  # wait for process exit and USB device release
  except Exception:
    pass
  bin_path = os.path.realpath(TEENSY_INTERFACE_BIN)
  if not os.path.isfile(bin_path):
    print(f"% start_teensy_interface: binary not found at {bin_path}")
    return False
  log_path = os.path.join(os.path.dirname(bin_path), "out_console.txt")
  log_file = open(log_path, "w")
  _ti_proc = subprocess.Popen(
      [bin_path, "-l", "-d"],
      cwd=os.path.dirname(bin_path),
      stdout=log_file,
      stderr=log_file,
  )
  print(f"% teensy_interface started (pid {_ti_proc.pid}), log: {log_path}")
  t.sleep(3.0)  # wait for USB + MQTT connection to fully establish
  if _ti_proc.poll() is not None:
    _ti_proc.wait()  # reap zombie
    print(f"% teensy_interface exited early (code {_ti_proc.returncode}) - check {log_path}")
    _ti_proc = None
    return False
  return True

def stop_teensy_interface():
  """Stop the teensy_interface subprocess we started (if any)."""
  global _ti_proc
  if _ti_proc is None:
    return
  if _ti_proc.poll() is None:
    print(f"% stopping teensy_interface (pid {_ti_proc.pid})")
    _ti_proc.send_signal(signal.SIGINT)
    try:
      _ti_proc.wait(timeout=3)
    except subprocess.TimeoutExpired:
      _ti_proc.kill()
      _ti_proc.wait()  # reap after force-kill
  else:
    _ti_proc.wait()  # process already exited; reap zombie
  _ti_proc = None
