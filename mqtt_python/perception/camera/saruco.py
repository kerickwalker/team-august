#!/usr/bin/env python3
"""
saruco.py
=============================================================================
ArUco marker detection module.

For each detected marker returns:
    - marker ID
    - range   (metres, distance from camera)
    - bearing (radians, + = left, - = right)
    - pixel centre (u, v)

Marker sizes (measure again on-site to confirm):
    Sorting center (ID 10-17) : 10 cm  = 0.10 m
    Luggage cubes  (ID 20,53) :  3.5 cm = 0.035 m
    Shuttle        (ID 5)     :  3.5 cm = 0.035 m

"""

import cv2
import numpy as np
import sys
import os

MARKER_SIZE_M = {
    10: 0.10, 11: 0.10, 12: 0.10, 13: 0.10,
    14: 0.10, 15: 0.10, 16: 0.10, 17: 0.10,
    20: 0.035, 53: 0.035,
     5: 0.035,
}
DEFAULT_MARKER_SIZE_M = 0.10

ARUCO_DICT = cv2.aruco.DICT_4X4_50


class SArucoDetector:
    ready       = False
    detections  = []
    debug_frame = None
    debug       = False

    _camera_matrix = None
    _dist_coeffs   = None
    _detector      = None
    _extrinsics    = None  # CameraExtrinsics instance — camera-to-robot rigid transform
    _id_to_role    = {}    # Populated from field_map in setup(); falls back to "unknown".
    _id_to_mobile  = {}    # id -> MobileArucoSpec for task markers (luggage, shuttle, …).

    def setup(self, params_path: str = '../calibration/camera_params.npz',
              debug: bool = False):
        self.debug = debug

        if not os.path.isfile(params_path):
            print(f"% SArucoDetector: calibration file not found: {params_path}")
            return

        data = np.load(params_path)
        self._camera_matrix = data['camera_matrix']
        self._dist_coeffs   = data['dist_coeffs']

        # Lazy import keeps the module standalone-runnable without numpy import order issues.
        from perception.camera.scamera_extrinsics import load as load_extrinsics
        self._extrinsics = load_extrinsics()

        # Lazy import: field_map is the single source of truth for marker roles.
        # If it's unavailable (e.g. running this file standalone), every detected
        # marker gets role="unknown" — detection itself stays operational.
        self._id_to_role, self._id_to_mobile = self._build_role_tables()

        aruco_dict     = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        aruco_params   = cv2.aruco.DetectorParameters()
        # Sub-pixel corner refinement noticeably improves PnP accuracy at long range.
        aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.ready = True
        print(f"% SArucoDetector: ready  known_ids={sorted(MARKER_SIZE_M.keys())}  "
              f"debug={'on' if debug else 'off'}")
        print(f"% SArucoDetector: extrinsics  {self._extrinsics}")
        if self._id_to_role:
            roles_summary = sorted(set(self._id_to_role.values()))
            print(f"% SArucoDetector: roles loaded for {len(self._id_to_role)} marker(s) "
                  f"({roles_summary})")
            if self._id_to_mobile:
                print(f"% SArucoDetector: mobile specs loaded for "
                      f"{len(self._id_to_mobile)} marker(s) "
                      f"(ids={sorted(self._id_to_mobile)})")
        else:
            print("% SArucoDetector: no roles loaded — every detection will be tagged 'unknown'")

    @staticmethod
    def _build_role_tables() -> tuple:
        """Build (id_to_role, id_to_mobile_spec) from the field map.

        Static localization markers come from `FIELD.all_aruco()`; task /
        mobile markers (luggage, shuttle, …) come from
        `FIELD.all_mobile_arucos()`. Same ID listed in both is a config
        error — caller is warned and the mobile entry wins (it carries the
        object-centre offset, which is the more useful piece downstream).
        """
        try:
            from field_map.field_map_2026 import FIELD
            from field_map.field_objects import MARKER_ROLE_UNKNOWN
        except Exception as e:
            print(f"% SArucoDetector: field map unavailable ({e!r}); roles will be 'unknown'")
            return {}, {}

        id_to_role: dict = {}
        id_to_mobile: dict = {}

        for m in FIELD.all_aruco():
            role = getattr(m, "role", None) or MARKER_ROLE_UNKNOWN
            id_to_role[int(m.id)] = role

        for spec in FIELD.all_mobile_arucos():
            mid = int(spec.id)
            if mid in id_to_role:
                print(f"% SArucoDetector: WARN id={mid} listed as static "
                      f"({id_to_role[mid]}) and mobile ({spec.role}); mobile wins")
            id_to_role[mid] = spec.role
            id_to_mobile[mid] = spec

        return id_to_role, id_to_mobile

    def process(self, frame):
        """
        Detect ArUco markers in a BGR frame.

        Returns list of dicts:
            id              : int
            role            : str   ("localization" | "luggage" | "shuttle" | …)
            range           : float (m, robot-frame XY plane)
            bearing         : float (rad, robot frame, + = left)
            u, v            : float (pixel centre)
            rvec            : (3,)  marker→camera rotation (axis-angle), camera frame
            tvec            : (3,)  marker translation, camera frame, m
            T_marker_rob    : (3,)  marker translation, robot frame, m  (extrinsics applied)
            R_marker_rob    : (3,3) marker→robot rotation matrix       (extrinsics applied)
            object_center_rob (opt): (3,) attached object's geometric centre in robot
                                     frame, m. Present only for markers with a
                                     MobileArucoSpec (e.g. cube centre behind the
                                     visible face).
            object_kind     (opt): str — copied from the spec ("cube", "platform", …)
            object_name     (opt): str — copied from the spec ("luggage_zoneA", …)
        """
        if not self.ready:
            return []

        corners, ids, _ = self._detector.detectMarkers(frame)
        self.detections = []

        if ids is None:
            if self.debug:
                self.debug_frame = self._draw_debug(frame, [], None, None, None)
            return []

        rvecs_all = []
        tvecs_all = []

        for i, marker_id in enumerate(ids.flatten()):
            marker_size = MARKER_SIZE_M.get(int(marker_id), DEFAULT_MARKER_SIZE_M)

            half = marker_size / 2.0
            obj_pts = np.array([
                [-half,  half, 0],
                [ half,  half, 0],
                [ half, -half, 0],
                [-half, -half, 0],
            ], dtype=np.float32)
            img_pts = corners[i][0].astype(np.float32)

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts,
                self._camera_matrix, self._dist_coeffs,
                flags=cv2.SOLVEPNP_IPPE_SQUARE)
            if not ok:
                continue

            rvec = rvec.flatten()
            tvec = tvec.flatten()

            # 1) marker pose in camera frame
            R_marker_cam, _ = cv2.Rodrigues(rvec)

            # 2) lift to robot body frame using the calibrated extrinsics
            T_marker_rob = self._extrinsics.transform_point(tvec)
            R_marker_rob = self._extrinsics.transform_rotation(R_marker_cam)

            # 3) range/bearing in the robot's horizontal plane (X-Y),
            #    so they're directly usable for navigation. + bearing = marker on robot's left.
            x_rob, y_rob, _z_rob = T_marker_rob
            range_m = float(np.hypot(x_rob, y_rob))
            bearing = float(np.arctan2(y_rob, x_rob))

            c   = corners[i][0]
            u_c = float(np.mean(c[:, 0]))
            v_c = float(np.mean(c[:, 1]))

            mid_int = int(marker_id)
            role = self._id_to_role.get(mid_int, "unknown")

            det = {
                "id":           mid_int,
                "role":         role,
                "range":        round(range_m, 4),
                "bearing":      round(bearing, 4),
                "u":            round(u_c, 1),
                "v":            round(v_c, 1),
                "rvec":         rvec,
                "tvec":         tvec,
                "T_marker_rob": T_marker_rob,
                "R_marker_rob": R_marker_rob,
            }

            spec = self._id_to_mobile.get(mid_int)
            if spec is not None:
                # Object centre = marker centre + R_marker_rob · offset_marker_frame.
                # The offset lives in the marker's own local frame (OpenCV ArUco
                # convention), so it is rotated by the marker→robot rotation
                # before adding to the marker translation.
                offset_marker = np.asarray(spec.object_offset_marker_frame, dtype=np.float64)
                object_center_rob = T_marker_rob + R_marker_rob @ offset_marker
                det["object_center_rob"] = object_center_rob
                if spec.object_kind:
                    det["object_kind"] = spec.object_kind
                if spec.name:
                    det["object_name"] = spec.name

            self.detections.append(det)
            rvecs_all.append(rvec)
            tvecs_all.append(tvec)

        if self.debug:
            self.debug_frame = self._draw_debug(
                frame, corners, ids, rvecs_all, tvecs_all)

        return self.detections

    def _draw_debug(self, frame, corners, ids, rvecs, tvecs):
        vis = frame.copy()

        if ids is None or len(ids) == 0:
            cv2.putText(vis, "No ArUco markers detected", (8, 30),
                        cv2.FONT_HERSHEY_PLAIN, 1.2, (0, 60, 220), 1)
            return vis

        cv2.aruco.drawDetectedMarkers(vis, corners, ids)

        for i, marker_id in enumerate(ids.flatten()):
            if rvecs is not None and tvecs is not None:
                marker_size = MARKER_SIZE_M.get(int(marker_id), DEFAULT_MARKER_SIZE_M)
                cv2.drawFrameAxes(vis, self._camera_matrix, self._dist_coeffs,
                                  rvecs[i], tvecs[i], marker_size * 0.5)

            det = self.detections[i] if i < len(self.detections) else None
            if det:
                c   = corners[i][0]
                u_c = int(np.mean(c[:, 0]))
                v_c = int(np.mean(c[:, 1]))
                label = (f"ID={det['id']}  "
                         f"r={det['range']:.2f}m  "
                         f"b={np.degrees(det['bearing']):+.1f}deg")
                (tw, th), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_PLAIN, 0.95, 1)
                cv2.rectangle(vis, (u_c - 2, v_c - th - 4),
                              (u_c + tw + 2, v_c + 2), (0, 0, 0), -1)
                cv2.putText(vis, label, (u_c, v_c - 4),
                            cv2.FONT_HERSHEY_PLAIN, 0.95, (0, 255, 100), 1)

        cv2.putText(vis, f"Detected: {len(ids)} marker(s)", (8, 22),
                    cv2.FONT_HERSHEY_PLAIN, 1.1, (200, 200, 0), 1)
        return vis

    def terminate(self):
        print("% SArucoDetector: terminated")


aruco_detector = SArucoDetector()


if __name__ == "__main__":
    import argparse
    import time

    parser = argparse.ArgumentParser(description="Live ArUco detection")
    parser.add_argument("--host",   default="localhost")
    parser.add_argument("--port",   type=int, default=7123)
    parser.add_argument("--params", default="../calibration/camera_params.npz")
    args = parser.parse_args()

    aruco_detector.setup(args.params, debug=True)
    if not aruco_detector.ready:
        sys.exit(1)

    stream_url = f"http://{args.host}:{args.port}/stream.mjpg"
    print(f"% Connecting to {stream_url} ...")
    cap = cv2.VideoCapture(stream_url)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    if not cap.isOpened():
        print("% ERROR: Could not open stream")
        sys.exit(1)

    print("% Q = quit")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.1)
            continue

        detections = aruco_detector.process(frame)

        for d in detections:
            print(f"  ID={d['id']:3d}  "
                  f"range={d['range']:.3f}m  "
                  f"bearing={np.degrees(d['bearing']):+.1f}deg  "
                  f"pixel=({d['u']:.0f},{d['v']:.0f})")

        if aruco_detector.debug_frame is not None:
            cv2.imshow("ArUco Detection", aruco_detector.debug_frame)

        if cv2.waitKey(1) & 0xFF in (ord('q'), 27):
            break

    cap.release()
    cv2.destroyAllWindows()