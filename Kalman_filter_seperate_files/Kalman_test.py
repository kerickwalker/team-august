from Kalman_class import KalmanFilter
import numpy as np
from math import cos, sin
from csv_loader import CSVDataLoader
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation, PillowWriter
from pathlib import Path

# Failure scenario selector (0 uses nominal dataset, 1-10 use generated failure datasets).
FAILURE_CASE = 1

FAILURE_DESCRIPTIONS = {
    0: "Nominal sensors (baseline sensor_measurements.csv)",
    1: "pos_x fault: large drift/noise",
    2: "pos_y fault: intermittent outlier spikes",
    3: "pos_z fault: elevated measurement noise",
    4: "velocity fault: positive bias",
    5: "yaw fault: positive bias",
    6: "angular_velocity fault: spike bursts/outliers",
    7: "pitch fault: elevated jitter/noise",
    8: "mixed fault: pos_x and velocity disturbance",
    9: "mixed fault: severe angular_velocity spikes with yaw disturbance",
    10: "mixed fault: pos_x/pos_y disturbance with secondary velocity/angular_velocity effects",
}


def resolve_measurement_file(case_id: int) -> str:
    """Return measurement CSV path for the selected failure case."""
    if case_id == 0:
        return "sensor_measurements.csv"
    if 1 <= case_id <= 10:
        return f"sensor_measurements_failure_{case_id:02d}.csv"
    raise ValueError("FAILURE_CASE must be in range [0, 10]")


# Define system parameters
L = 1.0  # Track width

# Initial state components
initial_px = 0.5 # Initial x (global) position
initial_py = 0.0 # Initial y (global) position
initial_pz = 0.0 # Initial z (global) position
initial_velocity = 0.5 # Initial linear velocity
initial_angular_velocity = 0.0 # Initial angular velocity
initial_yaw = 0.0 # Initial yaw
initial_pitch = 0.0 # Initial pitch
dt = 0.1  # Time step


def build_state_transition_matrix(yaw: float, pitch: float, dt_s: float) -> np.ndarray:
    """Build A using current estimated state (not fixed sensor angles)."""

    return np.array([
        [1.0, 0.0, 0.0, cos(pitch) * sin(yaw) * dt_s, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, cos(pitch) * cos(yaw) * dt_s, 0.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, sin(pitch) * dt_s, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, dt_s, 1.0, 0.0],
        [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, pitch],
    ])


A = build_state_transition_matrix(initial_yaw, initial_pitch, dt)  # State transition matrix (7x7)


B = np.array([
    [0.0, 0.0],
    [0.0, 0.0],
    [0.0, 0.0],
    [0.5, 0.5],
    [0.5 * L, -0.5 * L],
    [0.0, 0.0],
    [0.0, 0.0]
])  # Control input matrix (7x2)

H = np.eye(7)  # Measurement matrix (7x7)

# State order for all vectors/matrices:
# 0=pos_x, 1=pos_y, 2=pos_z, 3=velocity, 4=angular_velocity, 5=yaw, 6=pitch

# Process noise variance per state (Q diagonal). Edit each entry by hand.
q_diag = np.array([
    0.01,  # 0 pos_x process variance
    0.01,  # 1 pos_y process variance
    0.01,  # 2 pos_z process variance
    0.01,  # 3 velocity process variance
    0.01,  # 4 angular_velocity process variance
    0.01,  # 5 yaw process variance
    0.01,  # 6 pitch process variance
])

# Measurement noise variance per state (R diagonal). Edit each entry by hand.
r_diag = np.array([
    0.5,   # 0 pos_x measurement variance
    0.1,   # 1 pos_y measurement variance
    0.1,   # 2 pos_z measurement variance
    0.1,   # 3 velocity measurement variance
    0.1,   # 4 angular_velocity measurement variance
    0.1,   # 5 yaw measurement variance
    0.1,   # 6 pitch measurement variance
])

Q = np.diag(q_diag)
R = np.diag(r_diag)

# Create Kalman Filter instance
kf = KalmanFilter(A, B, H, Q, R)
kf.x = np.array([
    [initial_px],
    [initial_py],
    [initial_pz],
    [initial_velocity],
    [initial_angular_velocity],
    [initial_yaw],
    [initial_pitch],
], dtype=float)


loader = CSVDataLoader()

measurement_file = resolve_measurement_file(FAILURE_CASE)
failure_description = FAILURE_DESCRIPTIONS[FAILURE_CASE]

# Load measurements and controls from CSV files.
measurements = loader.load_measurements_7x1(
    measurement_file,
    columns=["pos_x", "pos_y", "pos_z", "velocity", "angular_velocity", "yaw", "pitch"],
)
control_inputs = loader.load_control_inputs_2x1("control_inputs.csv", left_column="u_left", right_column="u_right")
print(f"Loaded {len(measurements)} measurements and {len(control_inputs)} control inputs.")
print(f"Failure case {FAILURE_CASE}: {failure_description}")
print(f"Measurement source: {measurement_file}")

if len(measurements) != len(control_inputs):
    raise ValueError(f"{measurement_file} and control_inputs.csv row count must match")

if measurements and measurements[0].shape != (7, 1):
    raise ValueError("Measurements must be loaded as 7x1 vectors")

if control_inputs and control_inputs[0].shape != (2, 1):
    raise ValueError("Control inputs must be loaded as 2x1 vectors")

# Save estimates for plotting.
times = []
estimated_history = []
raw_history = []
pure_model_history = []

# Open-loop model state: propagated only by model equations and control inputs.
x_model = kf.x.copy()

# Run Kalman Filter with measurements and control inputs
for step, (z, u) in enumerate(zip(measurements, control_inputs)):
    # Pure model propagation (no measurement correction).
    model_yaw = float(x_model[5, 0])
    model_pitch = float(x_model[6, 0])
    A_model = build_state_transition_matrix(model_yaw, model_pitch, dt)
    x_model = A_model @ x_model + kf.B @ u
    pure_model_history.append(x_model.flatten())

    # Recompute A from current estimated state before prediction.
    yaw_hat = float(kf.x[5, 0])
    pitch_hat = float(kf.x[6, 0])
    kf.A = build_state_transition_matrix(yaw_hat, pitch_hat, dt)

    kf.predict(u)  # u is an explicit 2x1 control vector
    kf.update(z)  # Update with 7x1 measurement vector
    times.append(step * dt)
    estimated_history.append(kf.x.flatten())
    raw_history.append(z.flatten())
    print(f"Estimated state: {kf.x.flatten()}")

if estimated_history:
    estimated_history = np.array(estimated_history)
    raw_history = np.array(raw_history)
    pure_model_history = np.array(pure_model_history)

    # 3D trajectory GIF over time for position states.
    fig_3d = plt.figure(figsize=(8, 6))
    ax_3d = fig_3d.add_subplot(111, projection="3d")

    x_est = estimated_history[:, 0]
    y_est = estimated_history[:, 1]
    z_est = estimated_history[:, 2]
    x_model_vals = pure_model_history[:, 0]
    y_model_vals = pure_model_history[:, 1]
    z_model_vals = pure_model_history[:, 2]
    x_raw = raw_history[:, 0]
    y_raw = raw_history[:, 1]
    z_raw = raw_history[:, 2]

    line_est, = ax_3d.plot([], [], [], lw=1.8, color="tab:blue", label="Kalman estimate")
    point_est, = ax_3d.plot([], [], [], marker="o", color="tab:blue", markersize=4)
    line_model, = ax_3d.plot([], [], [], lw=1.4, color="tab:green", label="Pure model output")
    point_model, = ax_3d.plot([], [], [], marker="^", color="tab:green", markersize=4)
    line_raw, = ax_3d.plot([], [], [], lw=1.2, color="tab:orange", alpha=0.8, label="Raw sensor")
    point_raw, = ax_3d.plot([], [], [], marker="x", color="tab:orange", markersize=5)

    x_all = np.concatenate([x_est, x_model_vals, x_raw])
    y_all = np.concatenate([y_est, y_model_vals, y_raw])
    z_all = np.concatenate([z_est, z_model_vals, z_raw])

    x_pad = max(1e-6, 0.05 * (x_all.max() - x_all.min() if x_all.max() != x_all.min() else 1.0))
    y_pad = max(1e-6, 0.05 * (y_all.max() - y_all.min() if y_all.max() != y_all.min() else 1.0))
    z_pad = max(1e-6, 0.05 * (z_all.max() - z_all.min() if z_all.max() != z_all.min() else 1.0))

    ax_3d.set_xlim(x_all.min() - x_pad, x_all.max() + x_pad)
    ax_3d.set_ylim(y_all.min() - y_pad, y_all.max() + y_pad)
    ax_3d.set_zlim(z_all.min() - z_pad, z_all.max() + z_pad)
    ax_3d.set_xlabel("pos_x")
    ax_3d.set_ylabel("pos_y")
    ax_3d.set_zlabel("pos_z")
    ax_3d.grid(True, alpha=0.3)
    ax_3d.legend(loc="upper left")

    def animate(frame_idx: int):
        line_est.set_data(x_est[: frame_idx + 1], y_est[: frame_idx + 1])
        line_est.set_3d_properties(z_est[: frame_idx + 1])
        point_est.set_data([x_est[frame_idx]], [y_est[frame_idx]])
        point_est.set_3d_properties([z_est[frame_idx]])

        line_model.set_data(x_model_vals[: frame_idx + 1], y_model_vals[: frame_idx + 1])
        line_model.set_3d_properties(z_model_vals[: frame_idx + 1])
        point_model.set_data([x_model_vals[frame_idx]], [y_model_vals[frame_idx]])
        point_model.set_3d_properties([z_model_vals[frame_idx]])

        line_raw.set_data(x_raw[: frame_idx + 1], y_raw[: frame_idx + 1])
        line_raw.set_3d_properties(z_raw[: frame_idx + 1])
        point_raw.set_data([x_raw[frame_idx]], [y_raw[frame_idx]])
        point_raw.set_3d_properties([z_raw[frame_idx]])

        ax_3d.set_title(f"3D Position: Raw vs Pure Model vs Kalman (t = {times[frame_idx]:.2f}s)")
        return line_est, point_est, line_model, point_model, line_raw, point_raw

    animation = FuncAnimation(
        fig_3d,
        animate,
        frames=len(times),
        interval=100,
        blit=False,
        repeat=True,
    )

    gif_path = Path("estimated_position_trajectory.gif")
    animation.save(gif_path, writer=PillowWriter(fps=10))
    print(f"Saved 3D trajectory GIF: {gif_path.resolve()}")
    plt.close(fig_3d)

    # 2D XY path view for easier top-down trajectory inspection.
    fig_xy, ax_xy = plt.subplots(figsize=(7, 6))
    ax_xy.plot(estimated_history[:, 0], estimated_history[:, 1], color="tab:blue", lw=1.8, label="Kalman path")
    ax_xy.plot(pure_model_history[:, 0], pure_model_history[:, 1], color="tab:green", lw=1.4, label="Pure model path")
    ax_xy.plot(raw_history[:, 0], raw_history[:, 1], color="tab:orange", lw=1.2, alpha=0.8, label="Raw sensor path")

    ax_xy.scatter(estimated_history[0, 0], estimated_history[0, 1], color="green", marker="o", s=45, label="Start")
    ax_xy.scatter(estimated_history[-1, 0], estimated_history[-1, 1], color="red", marker="X", s=55, label="End")

    ax_xy.set_xlabel("pos_x")
    ax_xy.set_ylabel("pos_y")
    ax_xy.set_title("2D Robot Path (X-Y Plane)")
    ax_xy.grid(True, alpha=0.3)
    ax_xy.axis("equal")
    ax_xy.legend(loc="best")
    fig_xy.tight_layout()

    # Keep remaining states as time-domain dot plots.
    remaining_state_names = ["velocity", "angular_velocity", "yaw", "pitch"]
    remaining_indices = [3, 4, 5, 6]

    fig_2d, axes = plt.subplots(2, 2, figsize=(12, 8), sharex=True)
    axes = axes.flatten()

    for ax, state_name, idx in zip(axes, remaining_state_names, remaining_indices):
        ax.plot(times, estimated_history[:, idx], lw=1.5, label="Kalman")
        ax.plot(times, pure_model_history[:, idx], lw=1.2, label="Pure model")
        ax.scatter(times, raw_history[:, idx], s=10, alpha=0.6, label="Raw")
        ax.set_ylabel(state_name)
        ax.grid(True, alpha=0.3)
        ax.legend(loc="best")

    axes[2].set_xlabel("time [s]")
    axes[3].set_xlabel("time [s]")
    fig_2d.suptitle("Raw vs Pure Model vs Kalman (Non-Position States)")
    fig_2d.tight_layout()

    plt.show()
