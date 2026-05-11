import json
import platform
from paho.mqtt import client as mqtt_client
from spursuit import pursuit

# Path to the pre-generated trajectory CSV.
# Columns expected: x (forward), y (left), heading (CCW+ rad), cumulative_dist (m)
TRAJECTORY_CSV = "trajectory.csv"


class SPathFollow:
    """
    MQTT integration layer for the Pure Pursuit path-following controller.

    Responsibilities
    ----------------
    - Subscribes to robobot/kalman/state (own MQTT client, independent of uservice)
    - Parses the Kalman-filtered pose (x, y, yaw, velocity) from JSON
    - Provides pathControl(velocity) to arm / disarm the controller
    - update() is called from the mission loop at ~50 Hz; it computes a Pure Pursuit
      command and publishes it to robobot/cmd/ti as "rc <linvel> <turnrate>"

    Analogous to SEdge / sedge.py but driven by trajectory instead of line sensor.
    """

    def __init__(self):
        self._active        = False
        self._velocity      = 0.0
        # Kalman pose (written by MQTT callback, read by update())
        self._x             = 0.0
        self._y             = 0.0
        self._heading       = 0.0
        self._speed         = 0.0
        self._pose_received = False
        self._client        = None
        # Last velocity command sent (read by mission loop for status prints)
        self._last_linvel   = 0.0
        self._last_turnrate = 0.0

    def setup(self):
        """
        Connect a dedicated MQTT client to the broker and subscribe to the
        Kalman state topic.  Called once from uservice.setup() alongside
        the other subsystem setups.
        """
        # Mirror uservice.py's platform-aware client creation
        if platform.system() == "Windows":
            self._client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2)
        else:
            try:
                self._client = mqtt_client.Client(client_id="path-follow-in")
            except Exception:
                self._client = mqtt_client.Client(mqtt_client.CallbackAPIVersion.VERSION2)

        self._client.on_message = self._on_kalman_message
        try:
            self._client.connect("localhost", 1883)
            self._client.loop_start()
            self._client.subscribe("robobot/kalman/state", qos=1)
            print("% SPathFollow: subscribed to robobot/kalman/state")
        except Exception as e:
            print(f"% SPathFollow: WARNING — could not connect to broker: {e}")
            print("% SPathFollow: path-follow will wait for pose before sending commands")

    def _on_kalman_message(self, client, userdata, msg):
        """
        Parse the Kalman state JSON.  Handles two schemas published by the
        Kalman filter node:

          Schema A: {"x": {"x": ..., "y": ..., "yaw": ..., "velocity": ..., ...}}
          Schema B: {"position": {"x":..., "y":...}, "orientation": {"yaw":...},
                     "velocity": {"linear":..., "angular":...}}
        """
        try:
            data = json.loads(msg.payload.decode('utf-8'))
            if 'x' in data:
                s = data['x']
                self._x       = s.get('x', 0.0)
                self._y       = s.get('y', 0.0)
                self._heading = s.get('yaw', 0.0)
                self._speed   = s.get('velocity', 0.0)
            elif 'position' in data:
                self._x       = data['position'].get('x', 0.0)
                self._y       = data['position'].get('y', 0.0)
                self._heading = data.get('orientation', {}).get('yaw', 0.0)
                self._speed   = data.get('velocity', {}).get('linear', 0.0)
            self._pose_received = True
        except Exception as e:
            print(f"% SPathFollow: error parsing Kalman message: {e}")

    def pathControl(self, velocity: float):
        """
        Arm (velocity > 0) or disarm the controller.

        On arm:  loads the trajectory CSV if not already loaded, resets the
                 nearest-point index, and enables update() to send commands.
        On disarm: stops sending commands (caller must send rc 0 0 separately).
        """
        if velocity > 0.001:
            pursuit.load_trajectory(TRAJECTORY_CSV)
            pursuit.reset()
            self._active   = True
            self._velocity = velocity
        else:
            self._active   = False
            self._velocity = 0.0

    def update(self):
        """
        Compute and publish one Pure Pursuit command.
        Call from the mission loop at ~50 Hz while pathControl is armed.
        Does nothing if not active or pose not yet received.
        """
        if not self._active or not self._pose_received:
            return
        linvel, turnrate = pursuit.compute_command(
            self._x, self._y, self._heading, self._speed, self._velocity)
        self._last_linvel   = linvel
        self._last_turnrate = turnrate
        from uservice import service
        service.send("robobot/cmd/ti", f"rc {linvel:.3f} {turnrate:.3f}")

    def terminate(self):
        """Disarm and disconnect the MQTT client.  Called from uservice.terminate()."""
        self._active = False
        if self._client is not None:
            try:
                self._client.loop_stop()
                self._client.disconnect()
            except Exception:
                pass


# module-level singleton — import with: from spath_follow import path_follow
path_follow = SPathFollow()
