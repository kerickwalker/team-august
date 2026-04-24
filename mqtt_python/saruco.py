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

    def setup(self, params_path: str = None,
              debug: bool = False):
        self.debug = debug
        if params_path is None:
            params_path = os.path.join(os.path.dirname(__file__), 'calibration', 'camera_params.npz')
        if not os.path.isfile(params_path):
            print(f"% SArucoDetector: calibration file not found: {params_path}")
            return

        data = np.load(params_path)
        self._camera_matrix = data['camera_matrix']
        self._dist_coeffs   = data['dist_coeffs']

        aruco_dict     = cv2.aruco.getPredefinedDictionary(ARUCO_DICT)
        aruco_params   = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        self.ready = True
        print(f"% SArucoDetector: ready  known_ids={sorted(MARKER_SIZE_M.keys())}  "
              f"debug={'on' if debug else 'off'}")

    def process(self, frame):
        """
        Detect ArUco markers in a BGR frame.

        Returns list of dicts:
            id      : int
            range   : float  (m)
            bearing : float  (rad, + = left)
            u, v    : float  (pixel centre)
            rvec    : (3,)
            tvec    : (3,)
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

            range_m = float(np.linalg.norm(tvec))
            bearing = float(-np.arctan2(tvec[0], tvec[2]))

            c   = corners[i][0]
            u_c = float(np.mean(c[:, 0]))
            v_c = float(np.mean(c[:, 1]))

            self.detections.append({
                "id":      int(marker_id),
                "range":   round(range_m, 4),
                "bearing": round(bearing, 4),
                "u":       round(u_c, 1),
                "v":       round(v_c, 1),
                "rvec":    rvec,
                "tvec":    tvec,
            })
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