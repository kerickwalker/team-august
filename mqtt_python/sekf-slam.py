#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
import math
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Optional

import cv2
import numpy as np

from sground import ground
from ssources import build_source

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None

try:
    from paho.mqtt import client as mqtt_client
except Exception:
    mqtt_client = None


DEFAULT_SOURCE = "video"
DEFAULT_SESSION = Path(__file__).resolve().parent / "recordings" / "test1"
PARAMS_PATH = Path(__file__).resolve().parent / "calibration" / "camera_params.npz"

# Tape extraction
WHITE_V_MIN = 160
WHITE_S_MAX = 60
BINARY_THRESHOLD = 200
ROI_TOP_FRACTION = 0.45
MIN_BLOB_AREA = 120
MIN_CONTOUR_AREA = 150
MIN_ASPECT_RATIO = 1.5
MASK_OPEN_KERNEL_SIZE = 3
MASK_CLOSE_KERNEL_SIZE = 5
SHADOW_FILL_KERNEL_SIZE = 9

# Slope-aware 3D point extraction
TAPE_WIDTH_M = 0.02
WIDTH_SEARCH_LIMIT_PX = 40
MAX_LINE_FIT_ERROR_PX = 2.5
MIN_WIDTH_PIXELS_FOR_RANGE = 6.0
GROUND_Z_M = 0.0

# Harris corners
HARRIS_MAX_CORNERS = 40
HARRIS_QUALITY_LEVEL = 0.08
HARRIS_MIN_DISTANCE_PX = 12
HARRIS_BLOCK_SIZE = 3
HARRIS_K = 0.04

# From-scratch SLAM
ASSOCIATION_RADIUS_XY_M = 0.18
ASSOCIATION_RADIUS_Z_M = 0.18
POSE_BLEND = 0.35
MAX_POSE_CORRECTION_DIST_M = 0.50
MAX_POSE_CORRECTION_YAW_RAD = math.radians(20.0)
MIN_MATCHES_FOR_CORRECTION = 2
DEFAULT_START_X = 0.235
DEFAULT_START_Y = 4.775
DEFAULT_START_YAW = 0.0
VISION_TOPIC = "robobot/drive/T0/vision_pose"


@dataclass
class Landmark:
    landmark_id: int
    x: float
    y: float
    z: float
    count: int
    last_seen: int


def wrap_angle(angle_rad: float) -> float:
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def remove_small_blobs(mask: np.ndarray, area_threshold: int) -> np.ndarray:
    n_labels, labels, stats, _ = cv2.connectedComponentsWithStats(mask, connectivity=8)
    clean = np.zeros_like(mask)
    for label in range(1, n_labels):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area >= area_threshold:
            clean[labels == label] = 255
    return clean


def detect_harris_points(mask_roi: np.ndarray) -> list[tuple[float, float]]:
    corners = cv2.goodFeaturesToTrack(
        mask_roi,
        maxCorners=HARRIS_MAX_CORNERS,
        qualityLevel=HARRIS_QUALITY_LEVEL,
        minDistance=HARRIS_MIN_DISTANCE_PX,
        mask=mask_roi,
        blockSize=HARRIS_BLOCK_SIZE,
        useHarrisDetector=True,
        k=HARRIS_K,
    )
    if corners is None:
        return []
    return [tuple(pt[0]) for pt in corners]


def contour_line_geometry(contour: np.ndarray) -> Optional[dict]:
    if len(contour) < 5:
        return None

    fit = cv2.fitLine(contour, cv2.DIST_L2, 0, 0.01, 0.01)
    vx = float(fit[0, 0])
    vy = float(fit[1, 0])
    x0 = float(fit[2, 0])
    y0 = float(fit[3, 0])
    tangent = np.array([vx, vy], dtype=float)
    tangent_norm = float(np.linalg.norm(tangent))
    if tangent_norm < 1e-6:
        return None
    tangent /= tangent_norm
    width_axis = np.array([-tangent[1], tangent[0]], dtype=float)

    pts = contour.reshape(-1, 2).astype(float)
    offsets = pts - np.array([x0, y0], dtype=float)
    fit_error = float(np.mean(np.abs(offsets[:, 0] * tangent[1] - offsets[:, 1] * tangent[0])))
    if fit_error > MAX_LINE_FIT_ERROR_PX:
        return None

    return {"width_axis": width_axis, "fit_error": fit_error}


def measure_tape_edges(
    mask_roi: np.ndarray, point_roi: tuple[float, float], width_axis: np.ndarray
) -> Optional[tuple[tuple[float, float], tuple[float, float], float]]:
    center = np.array(point_roi, dtype=float)
    height, width = mask_roi.shape[:2]

    def walk(sign: float) -> Optional[tuple[float, float]]:
        last_inside = None
        for step in range(WIDTH_SEARCH_LIMIT_PX + 1):
            probe = center + sign * width_axis * step
            x_px = int(round(probe[0]))
            y_px = int(round(probe[1]))
            if x_px < 0 or x_px >= width or y_px < 0 or y_px >= height:
                break
            if mask_roi[y_px, x_px] > 0:
                last_inside = (float(x_px), float(y_px))
            elif last_inside is not None:
                break
        return last_inside

    edge_pos = walk(+1.0)
    edge_neg = walk(-1.0)
    if edge_pos is None or edge_neg is None:
        return None

    width_px = float(np.hypot(edge_pos[0] - edge_neg[0], edge_pos[1] - edge_neg[1]))
    if width_px < MIN_WIDTH_PIXELS_FOR_RANGE:
        return None
    return edge_neg, edge_pos, width_px


def estimate_range_from_tape_width(
    center_global: tuple[float, float],
    edge0_global: tuple[float, float],
    edge1_global: tuple[float, float],
) -> Optional[float]:
    okc, rcx, rcy, rcz = ground.pixel_ray_robot(center_global[0], center_global[1])
    ok0, r0x, r0y, r0z = ground.pixel_ray_robot(edge0_global[0], edge0_global[1])
    ok1, r1x, r1y, r1z = ground.pixel_ray_robot(edge1_global[0], edge1_global[1])
    if not (okc and ok0 and ok1):
        return None

    center_ray = np.array([rcx, rcy, rcz], dtype=float)
    ray0 = np.array([r0x, r0y, r0z], dtype=float)
    ray1 = np.array([r1x, r1y, r1z], dtype=float)

    dot0 = float(np.dot(center_ray, ray0))
    dot1 = float(np.dot(center_ray, ray1))
    if dot0 <= 1e-6 or dot1 <= 1e-6:
        return None

    span = ray0 / dot0 - ray1 / dot1
    span_norm = float(np.linalg.norm(span))
    if span_norm < 1e-6:
        return None
    return TAPE_WIDTH_M / span_norm


def project_pixel_to_3d(
    u_px: float,
    v_px: float,
    pitch_rad: float,
    edge0_global: Optional[tuple[float, float]] = None,
    edge1_global: Optional[tuple[float, float]] = None,
) -> Optional[tuple[float, float, float]]:
    # sground uses X=forward, Y=left, Z=up.
    # Convert to X=forward, Y=right, Z=up for this script.
    ground.set_robot_pitch(pitch_rad)

    if edge0_global is not None and edge1_global is not None:
        distance_m = estimate_range_from_tape_width((u_px, v_px), edge0_global, edge1_global)
        if distance_m is not None:
            ok, x_forward_m, y_left_m, z_up_m = ground.ray_at_distance(u_px, v_px, distance_m)
            if ok:
                return (x_forward_m, -y_left_m, z_up_m)

    ok, x_forward_m, y_left_m = ground.pixel_to_ground_at_z(u_px, v_px, GROUND_Z_M)
    if not ok:
        return None
    return (x_forward_m, -y_left_m, GROUND_Z_M)


def add_status_lines(frame: np.ndarray, lines: list[str]) -> None:
    for idx, line in enumerate(lines):
        cv2.putText(
            frame,
            line,
            (8, 20 + idx * 18),
            cv2.FONT_HERSHEY_PLAIN,
            1.1,
            (0, 255, 255),
            1,
        )


class TapeFeatureExtractor:
    def extract(
        self, frame: np.ndarray, pitch_rad: float
    ) -> tuple[np.ndarray, list[tuple[float, float, float]], list[tuple[float, float]]]:
        img_h, img_w = frame.shape[:2]
        roi_top = int(img_h * ROI_TOP_FRACTION)
        roi = frame[roi_top:, :]

        open_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (MASK_OPEN_KERNEL_SIZE, MASK_OPEN_KERNEL_SIZE)
        )
        close_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (MASK_CLOSE_KERNEL_SIZE, MASK_CLOSE_KERNEL_SIZE)
        )
        shadow_fill_kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (SHADOW_FILL_KERNEL_SIZE, SHADOW_FILL_KERNEL_SIZE)
        )

        blurred = cv2.GaussianBlur(roi, (5, 5), 0)
        hsv = cv2.cvtColor(blurred, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, (0, 0, WHITE_V_MIN), (180, WHITE_S_MAX, 255))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, open_kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, close_kernel)
        mask = remove_small_blobs(mask, MIN_BLOB_AREA)

        white_lines = cv2.bitwise_and(roi, roi, mask=mask)
        gray_white_lines = cv2.cvtColor(white_lines, cv2.COLOR_BGR2GRAY)
        _, thresholded = cv2.threshold(
            gray_white_lines, BINARY_THRESHOLD, 255, cv2.THRESH_BINARY
        )
        thresholded = cv2.morphologyEx(thresholded, cv2.MORPH_CLOSE, shadow_fill_kernel)
        thresholded = cv2.medianBlur(thresholded, 5)
        thresholded = remove_small_blobs(thresholded, MIN_BLOB_AREA)

        contours, _ = cv2.findContours(
            thresholded, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        filtered_roi_mask = np.zeros_like(thresholded)
        accepted_contours: list[tuple[np.ndarray, Optional[dict]]] = []
        for contour in contours:
            if cv2.contourArea(contour) < MIN_CONTOUR_AREA:
                continue
            _, (w_px, h_px), _ = cv2.minAreaRect(contour)
            short_px = min(w_px, h_px)
            long_px = max(w_px, h_px)
            if short_px < 2.0 or long_px / short_px < MIN_ASPECT_RATIO:
                continue

            cv2.drawContours(filtered_roi_mask, [contour], -1, 255, thickness=-1)
            accepted_contours.append((contour, contour_line_geometry(contour)))

        filtered_mask = np.zeros((img_h, img_w), dtype=np.uint8)
        filtered_mask[roi_top:, :] = filtered_roi_mask
        vis = cv2.bitwise_and(frame, frame, mask=filtered_mask)

        harris_corners_roi = detect_harris_points(filtered_roi_mask)
        local_points_3d: list[tuple[float, float, float]] = []
        corner_pixels: list[tuple[float, float]] = []

        for u_roi, v_roi in harris_corners_roi:
            u_global = float(u_roi)
            v_global = float(v_roi + roi_top)
            edge0_global = None
            edge1_global = None

            for contour, line_geom in accepted_contours:
                if cv2.pointPolygonTest(contour, (float(u_roi), float(v_roi)), False) < 0:
                    continue
                if line_geom is not None:
                    edge_pair = measure_tape_edges(
                        filtered_roi_mask, (u_roi, v_roi), line_geom["width_axis"]
                    )
                    if edge_pair is not None:
                        edge0_roi, edge1_roi, _ = edge_pair
                        edge0_global = (edge0_roi[0], edge0_roi[1] + roi_top)
                        edge1_global = (edge1_roi[0], edge1_roi[1] + roi_top)
                break

            point_3d = project_pixel_to_3d(
                u_global, v_global, pitch_rad, edge0_global=edge0_global, edge1_global=edge1_global
            )
            if point_3d is None:
                continue

            local_points_3d.append(point_3d)
            corner_pixels.append((u_global, v_global))
            cv2.circle(vis, (int(round(u_global)), int(round(v_global))), 4, (0, 0, 255), -1)

        return vis, local_points_3d, corner_pixels


class ScratchTapeSlam:
    def __init__(self, mapping_enabled: bool = True):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.initialized = False
        self.mapping_enabled = mapping_enabled
        self.path: list[tuple[float, float, float]] = []
        self.landmarks: list[Landmark] = []
        self.next_landmark_id = 0

    def init_pose(self, x: float, y: float, yaw: float) -> None:
        self.x = float(x)
        self.y = float(y)
        self.yaw = float(yaw)
        self.initialized = True
        self.path = [(self.x, self.y, 0.0)]

    def predict(self, v: float, omega: float, dt: float) -> None:
        if not self.initialized:
            return

        dt = max(0.0, min(float(dt), 0.2))
        self.x += float(v) * math.cos(self.yaw) * dt
        self.y += float(v) * math.sin(self.yaw) * dt
        self.yaw = wrap_angle(self.yaw + float(omega) * dt)
        self.path.append((self.x, self.y, 0.0))

    def load_map(self, path: Path) -> None:
        with open(path, "r", encoding="utf-8") as fh:
            payload = json.load(fh)
        self.landmarks = [
            Landmark(
                landmark_id=int(item["landmark_id"]),
                x=float(item["x"]),
                y=float(item["y"]),
                z=float(item["z"]),
                count=int(item.get("count", 1)),
                last_seen=int(item.get("last_seen", -1)),
            )
            for item in payload.get("landmarks", [])
        ]
        self.next_landmark_id = (
            max((landmark.landmark_id for landmark in self.landmarks), default=-1) + 1
        )

    def robot_point_to_world(self, point_local: tuple[float, float, float]) -> np.ndarray:
        x_fwd, y_right, z_up = point_local
        x_world = self.x + math.cos(self.yaw) * x_fwd + math.sin(self.yaw) * y_right
        y_world = self.y + math.sin(self.yaw) * x_fwd - math.cos(self.yaw) * y_right
        return np.array([x_world, y_world, z_up], dtype=float)

    @staticmethod
    def _local_point_std_xy(point_local: tuple[float, float, float]) -> np.ndarray:
        x_fwd, y_right, _ = point_local
        return np.array([x_fwd, -y_right], dtype=float)

    def _nearest_landmark(self, world_point: np.ndarray, used_ids: set[int]) -> Optional[int]:
        best_idx = None
        best_dist = None
        for idx, landmark in enumerate(self.landmarks):
            if landmark.landmark_id in used_ids:
                continue
            dz = abs(landmark.z - world_point[2])
            if dz > ASSOCIATION_RADIUS_Z_M:
                continue
            dist_xy = math.hypot(landmark.x - world_point[0], landmark.y - world_point[1])
            if dist_xy > ASSOCIATION_RADIUS_XY_M:
                continue
            if best_dist is None or dist_xy < best_dist:
                best_idx = idx
                best_dist = dist_xy
        return best_idx

    def _estimate_pose_from_matches(
        self,
        local_points: list[tuple[float, float, float]],
        landmark_indices: list[int],
    ) -> Optional[tuple[float, float, float]]:
        if len(local_points) < MIN_MATCHES_FOR_CORRECTION:
            return None

        src = np.array([self._local_point_std_xy(p) for p in local_points], dtype=float)
        dst = np.array(
            [[self.landmarks[idx].x, self.landmarks[idx].y] for idx in landmark_indices],
            dtype=float,
        )
        if src.shape[0] < MIN_MATCHES_FOR_CORRECTION:
            return None

        src_centroid = src.mean(axis=0)
        dst_centroid = dst.mean(axis=0)
        src_centered = src - src_centroid
        dst_centered = dst - dst_centroid

        h_mat = src_centered.T @ dst_centered
        u_mat, _, vt_mat = np.linalg.svd(h_mat)
        rot = vt_mat.T @ u_mat.T
        if np.linalg.det(rot) < 0:
            vt_mat[1, :] *= -1
            rot = vt_mat.T @ u_mat.T

        yaw_est = math.atan2(rot[1, 0], rot[0, 0])
        trans = dst_centroid - rot @ src_centroid
        return float(trans[0]), float(trans[1]), wrap_angle(float(yaw_est))

    def _correct_pose_from_matches(
        self, local_points: list[tuple[float, float, float]]
    ) -> int:
        matched_local: list[tuple[float, float, float]] = []
        matched_landmark_indices: list[int] = []
        used_ids: set[int] = set()

        for point_local in local_points:
            world_guess = self.robot_point_to_world(point_local)
            landmark_idx = self._nearest_landmark(world_guess, used_ids)
            if landmark_idx is None:
                continue
            matched_local.append(point_local)
            matched_landmark_indices.append(landmark_idx)
            used_ids.add(self.landmarks[landmark_idx].landmark_id)

        pose_est = self._estimate_pose_from_matches(matched_local, matched_landmark_indices)
        if pose_est is None:
            return 0

        est_x, est_y, est_yaw = pose_est
        correction_dist = math.hypot(est_x - self.x, est_y - self.y)
        correction_yaw = abs(wrap_angle(est_yaw - self.yaw))
        if correction_dist > MAX_POSE_CORRECTION_DIST_M or correction_yaw > MAX_POSE_CORRECTION_YAW_RAD:
            return 0

        self.x += POSE_BLEND * (est_x - self.x)
        self.y += POSE_BLEND * (est_y - self.y)
        self.yaw = wrap_angle(self.yaw + POSE_BLEND * wrap_angle(est_yaw - self.yaw))
        return len(matched_local)

    def measurement_scale(self, n_points: int, matched_count: int) -> float:
        if n_points <= 0:
            return 100.0

        scale = 1.0
        if n_points < 6:
            scale += (6 - n_points) * 8.0
        elif n_points < 12:
            scale += (12 - n_points) * 1.5

        if matched_count < MIN_MATCHES_FOR_CORRECTION:
            scale += (MIN_MATCHES_FOR_CORRECTION - matched_count) * 12.0

        match_ratio = matched_count / max(float(n_points), 1.0)
        if match_ratio < 0.35:
            scale += (0.35 - match_ratio) * 40.0

        if not self.landmarks:
            scale += 25.0
        return min(scale, 100.0)

    def update(
        self, local_points: list[tuple[float, float, float]], frame_idx: int
    ) -> tuple[int, int, list[np.ndarray], float]:
        matched_for_correction = self._correct_pose_from_matches(local_points)

        used_ids: set[int] = set()
        matched_count = 0
        new_count = 0
        world_points: list[np.ndarray] = []

        for point_local in local_points:
            world_point = self.robot_point_to_world(point_local)
            world_points.append(world_point)
            landmark_idx = self._nearest_landmark(world_point, used_ids)
            if landmark_idx is None:
                if self.mapping_enabled:
                    self.landmarks.append(
                        Landmark(
                            landmark_id=self.next_landmark_id,
                            x=float(world_point[0]),
                            y=float(world_point[1]),
                            z=float(world_point[2]),
                            count=1,
                            last_seen=frame_idx,
                        )
                    )
                    self.next_landmark_id += 1
                    new_count += 1
                continue

            landmark = self.landmarks[landmark_idx]
            used_ids.add(landmark.landmark_id)
            matched_count += 1
            if self.mapping_enabled:
                total = landmark.count + 1
                landmark.x = (landmark.x * landmark.count + float(world_point[0])) / total
                landmark.y = (landmark.y * landmark.count + float(world_point[1])) / total
                landmark.z = (landmark.z * landmark.count + float(world_point[2])) / total
                landmark.count = total
                landmark.last_seen = frame_idx

        if matched_for_correction > matched_count:
            matched_count = matched_for_correction
        return matched_count, new_count, world_points, self.measurement_scale(
            len(local_points), matched_count
        )

    def pose_tuple(self) -> tuple[float, float, float]:
        return self.x, self.y, self.yaw

    def save_map(self, path: Path) -> None:
        payload = {
            "version": 1,
            "pose": {"x": self.x, "y": self.y, "yaw": self.yaw},
            "start_pose": {
                "x": DEFAULT_START_X,
                "y": DEFAULT_START_Y,
                "yaw": DEFAULT_START_YAW,
            },
            "landmarks": [asdict(landmark) for landmark in self.landmarks],
            "path": self.path,
        }
        path.parent.mkdir(parents=True, exist_ok=True)
        with open(path, "w", encoding="utf-8") as fh:
            json.dump(payload, fh, indent=2)


class LiveMap3DViewer:
    def __init__(self, enabled: bool = True):
        self.enabled = enabled and plt is not None
        self.fig = None
        self.ax = None
        self.path_line = None
        self.map_scatter = None
        self.obs_scatter = None
        self.pose_scatter = None
        if enabled and plt is None:
            print("% 3D viewer disabled: matplotlib not available")

    def setup(self) -> None:
        if not self.enabled:
            return

        plt.ion()
        self.fig = plt.figure(figsize=(9, 7))
        self.ax = self.fig.add_subplot(111, projection="3d")
        self.ax.set_title("Scratch Tape SLAM 3D Map")
        self.ax.set_xlabel("X world (m)")
        self.ax.set_ylabel("Y world (m)")
        self.ax.set_zlabel("Z world (m)")
        self.ax.view_init(elev=35, azim=-55)
        self.map_scatter = self.ax.scatter([], [], [], c="cyan", s=12, depthshade=True)
        self.obs_scatter = self.ax.scatter([], [], [], c="red", s=20, depthshade=True)
        self.pose_scatter = self.ax.scatter([], [], [], c="lime", s=60, depthshade=True)
        (self.path_line,) = self.ax.plot([], [], [], color="yellow", linewidth=1.5)

    def update(
        self,
        landmarks: list[Landmark],
        world_observations: list[np.ndarray],
        path: list[tuple[float, float, float]],
        pose: tuple[float, float, float],
        frame_idx: int,
    ) -> None:
        if not self.enabled or self.ax is None or frame_idx % 2 != 0:
            return

        if landmarks:
            xs = [lm.x for lm in landmarks]
            ys = [lm.y for lm in landmarks]
            zs = [lm.z for lm in landmarks]
        else:
            xs, ys, zs = [], [], []
        self.map_scatter._offsets3d = (xs, ys, zs)

        if world_observations:
            obs_x = [float(pt[0]) for pt in world_observations]
            obs_y = [float(pt[1]) for pt in world_observations]
            obs_z = [float(pt[2]) for pt in world_observations]
        else:
            obs_x, obs_y, obs_z = [], [], []
        self.obs_scatter._offsets3d = (obs_x, obs_y, obs_z)

        if path:
            path_x = [p[0] for p in path]
            path_y = [p[1] for p in path]
            path_z = [p[2] for p in path]
            self.path_line.set_data(path_x, path_y)
            self.path_line.set_3d_properties(path_z)
        else:
            self.path_line.set_data([], [])
            self.path_line.set_3d_properties([])

        self.pose_scatter._offsets3d = ([pose[0]], [pose[1]], [0.0])

        all_x = xs + obs_x + [pose[0]]
        all_y = ys + obs_y + [pose[1]]
        all_z = zs + obs_z + [0.0]
        if all_x and all_y and all_z:
            margin_xy = 0.5
            margin_z = 0.2
            self.ax.set_xlim(min(all_x) - margin_xy, max(all_x) + margin_xy)
            self.ax.set_ylim(min(all_y) - margin_xy, max(all_y) + margin_xy)
            self.ax.set_zlim(min(0.0, min(all_z) - margin_z), max(0.2, max(all_z) + margin_z))

        plt.pause(0.001)

    def close(self) -> None:
        if self.enabled and self.fig is not None:
            plt.close(self.fig)


class VisionPosePublisher:
    def __init__(self, host: str, enabled: bool):
        self.enabled = bool(enabled and mqtt_client is not None and host)
        self.host = host
        self.client = None
        if enabled and mqtt_client is None:
            print("% Vision publisher disabled: paho-mqtt not available")

    def setup(self) -> None:
        if not self.enabled:
            return
        try:
            if hasattr(mqtt_client, "CallbackAPIVersion"):
                self.client = mqtt_client.Client(
                    callback_api_version=mqtt_client.CallbackAPIVersion.VERSION1
                )
            else:
                self.client = mqtt_client.Client()
            self.client.connect(self.host, 1883, keepalive=30)
            self.client.loop_start()
            print(f"% Vision publisher connected to MQTT at {self.host}")
        except Exception as exc:
            print(f"% Vision publisher disabled: {exc}")
            self.enabled = False
            self.client = None

    def publish_pose(
        self, x: float, y: float, yaw: float, pitch: float, r_scale: float
    ) -> None:
        if not self.enabled or self.client is None:
            return
        msg = (
            f"{time.time():.6f} "
            f"{x:.4f} {y:.4f} nan {yaw:.4f} {pitch:.4f} {r_scale:.4f}"
        )
        self.client.publish(VISION_TOPIC, msg)

    def close(self) -> None:
        if self.client is not None:
            try:
                self.client.loop_stop()
                self.client.disconnect()
            except Exception:
                pass


def default_map_output(args) -> Path:
    if getattr(args, "map", ""):
        return Path(args.map)
    if args.source in {"video", "images"} and args.path:
        return Path(args.path) / "scratch_tape_map.json"
    return Path(__file__).resolve().parent / "scratch_tape_map.json"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="From-scratch tape SLAM with 3D viewer")
    parser.add_argument("--mode", default="map", choices=["map", "localize"])
    parser.add_argument("--source", default=DEFAULT_SOURCE, choices=["live", "video", "images", "image"])
    parser.add_argument("--path", default=str(DEFAULT_SESSION))
    parser.add_argument("--host", default="localhost")
    parser.add_argument("--cam-port", type=int, default=7123)
    parser.add_argument("--speed", type=float, default=1.0)
    parser.add_argument("--map", default="")
    parser.add_argument("--save-map", default="")
    parser.add_argument("--publish-vision", action="store_true")
    parser.add_argument("--mqtt-host", default="")
    parser.add_argument("--no-3d", action="store_true")
    return parser


def main() -> None:
    args = build_parser().parse_args()

    ground.setup(str(PARAMS_PATH))
    if not ground.ready:
        raise RuntimeError(f"Could not load camera calibration: {PARAMS_PATH}")

    print("% Scratch tape SLAM")
    print(f"% Mode        : {args.mode}")
    print(f"% Source      : {args.source}")
    if args.path:
        print(f"% Path        : {args.path}")
    print(f"% Camera intr : {PARAMS_PATH}")
    print(
        f"% Extrinsics  : height={ground.camera_height:.3f}m  "
        f"tilt={ground.camera_tilt:.1f}deg  roll={ground.camera_roll:.1f}deg"
    )

    source = build_source(args)
    source.setup()

    extractor = TapeFeatureExtractor()
    slam = ScratchTapeSlam(mapping_enabled=args.mode == "map")
    viewer = LiveMap3DViewer(enabled=not args.no_3d)
    viewer.setup()
    mqtt_host = args.mqtt_host or (args.host if args.publish_vision else "")
    publisher = VisionPosePublisher(mqtt_host, enabled=args.publish_vision)
    publisher.setup()

    map_output = Path(args.save_map) if args.save_map else default_map_output(args)
    print(f"% Map file    : {map_output}")
    if args.mode == "localize":
        if not map_output.exists():
            raise FileNotFoundError(f"Localization map not found: {map_output}")
        slam.load_map(map_output)
        print(f"% Loaded map: {map_output}  landmarks={len(slam.landmarks)}")

    try:
        for info in source.frames():
            if not slam.initialized:
                slam.init_pose(DEFAULT_START_X, DEFAULT_START_Y, DEFAULT_START_YAW)

            slam.predict(info.v, info.omega, info.dt)
            pitch_rad = info.ref_pitch if info.has_ref else 0.0
            vis, local_points_3d, corner_pixels = extractor.extract(info.frame, pitch_rad)
            matched_count, new_count, world_points, measurement_scale = slam.update(
                local_points_3d, info.frame_idx
            )
            pose_x, pose_y, pose_yaw = slam.pose_tuple()

            if args.publish_vision and args.mode == "localize" and slam.initialized:
                publisher.publish_pose(
                    pose_x, pose_y, pose_yaw, pitch_rad, measurement_scale
                )

            add_status_lines(
                vis,
                [
                    f"frame={info.frame_idx}  src={info.source_name}",
                    f"mode={args.mode}  start=({DEFAULT_START_X:.3f}, {DEFAULT_START_Y:.3f}, {DEFAULT_START_YAW:.3f})",
                    f"pose x={pose_x:.2f} y={pose_y:.2f} yaw={math.degrees(pose_yaw):.1f}deg",
                    f"pitch={math.degrees(pitch_rad):.1f}deg  feats={len(local_points_3d)}  corners={len(corner_pixels)}",
                    f"landmarks={len(slam.landmarks)}  matched={matched_count}  new={new_count}  Rscale={measurement_scale:.1f}",
                ],
            )

            viewer.update(slam.landmarks, world_points, slam.path, slam.pose_tuple(), info.frame_idx)
            cv2.imshow("Scratch Tape SLAM", vis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break

    finally:
        source.release()
        publisher.close()
        viewer.close()
        cv2.destroyAllWindows()
        if args.mode == "map":
            slam.save_map(map_output)
            print(f"% Saved map: {map_output}")


if __name__ == "__main__":
    main()
