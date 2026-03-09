import cv2
import numpy as np
import glob
import os
import argparse
from datetime import datetime

# ─── CHECKERBOARD CONFIG ──────────────────────────────────────────────────────
BOARD_W = 7
BOARD_H = 10
SQUARE_SIZE = 0.025  

RMS_TARGET = 0.5   # pixels. < 0.5 is excellent, < 1.0 is acceptable

OUTLIER_SIGMA = 1.8
# ─────────────────────────────────────────────────────────────────────────────


def find_corners(images_dir: str, board_size: tuple, criteria):
    pattern = os.path.join(images_dir, "*.jpg")
    paths = sorted(glob.glob(pattern))
    if not paths:
        pattern = os.path.join(images_dir, "*.png")
        paths = sorted(glob.glob(pattern))

    print(f"% Found {len(paths)} images in '{images_dir}'")

    # 3D object points for one board pose
    objp = np.zeros((board_size[0] * board_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:board_size[0], 0:board_size[1]].T.reshape(-1, 2)
    objp *= SQUARE_SIZE

    objpoints = []
    imgpoints = []
    used_paths = []
    img_size = None

    for path in paths:
        img = cv2.imread(path)
        if img is None:
            print(f"%   SKIP (failed to load): {os.path.basename(path)}")
            continue

        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        if img_size is None:
            img_size = gray.shape[::-1]  # (w, h), set once 
        elif img_size != gray.shape[::-1]:
            print(f"%   SKIP (resolution mismatch {gray.shape[::-1]} != {img_size}): "
                  f"{os.path.basename(path)}")
            continue

        cb_flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                    cv2.CALIB_CB_NORMALIZE_IMAGE)
        # cb_flags |= cv2.CALIB_CB_FILTER_QUADS  # uncom. if detection is unreliable
        found, corners = cv2.findChessboardCorners(gray, board_size, cb_flags)
        if not found:
            print(f"%   SKIP (no corners): {os.path.basename(path)}")
            continue

        # Subpixel refinement —> critical for accuracy
        corners_refined = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1), criteria)

        objpoints.append(objp)
        imgpoints.append(corners_refined)
        used_paths.append(path)
        print(f"%   OK: {os.path.basename(path)}")

    return objpoints, imgpoints, used_paths, img_size


def compute_per_image_errors(objpoints, imgpoints, rvecs, tvecs,
                              camera_matrix, dist_coeffs):
    # Per-image mean reprojection errors
    errors = []
    for op, ip, rv, tv in zip(objpoints, imgpoints, rvecs, tvecs):
        projected, _ = cv2.projectPoints(op, rv, tv, camera_matrix, dist_coeffs)
        err = cv2.norm(ip, projected, cv2.NORM_L2) / len(projected)
        errors.append(err)
    return np.array(errors)


def calibrate_iterative(objpoints, imgpoints, used_paths, img_size, criteria):
    obj = list(objpoints)
    img = list(imgpoints)
    paths = list(used_paths)

    calib_flags = 0

    iteration = 0
    while True:
        rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
            obj, img, img_size, None, None,
            flags=calib_flags
        )
        per_err = compute_per_image_errors(obj, img, rvecs, tvecs,
                                           camera_matrix, dist_coeffs)
        mean_err = per_err.mean()

        print(f"%   [iter {iteration}] RMS={rms:.4f}px  "
              f"mean_per_img={mean_err:.4f}px  n={len(obj)}")

        if rms <= RMS_TARGET or len(obj) <= 10:
            break

        # Remove worst outlier image if it's beyond OUTLIER_SIGMA * std
        threshold = mean_err + OUTLIER_SIGMA * per_err.std()
        worst_idx = int(per_err.argmax())
        if per_err[worst_idx] > threshold:
            print(f"%     Removing outlier: {os.path.basename(paths[worst_idx])} "
                  f"(err={per_err[worst_idx]:.4f}px > threshold={threshold:.4f}px)")
            obj.pop(worst_idx)
            img.pop(worst_idx)
            paths.pop(worst_idx)
        else:
            break  

        iteration += 1

    final_per_err = compute_per_image_errors(obj, img, rvecs, tvecs,
                                             camera_matrix, dist_coeffs)
    return camera_matrix, dist_coeffs, rvecs, tvecs, final_per_err, paths, rms


def quality_report(camera_matrix, dist_coeffs, rms, per_errors, paths,
                   img_size, out_dir):
    # calibration quality report
    lines = []
    lines.append("=" * 60)
    lines.append("CAMERA CALIBRATION REPORT")
    lines.append(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    lines.append("=" * 60)
    lines.append(f"\nCheckerboard: {BOARD_W}×{BOARD_H} inner corners, "
                 f"square={SQUARE_SIZE*1000:.1f}mm")
    lines.append(f"Image size:   {img_size[0]}×{img_size[1]} px")
    lines.append(f"Images used:  {len(paths)}")
    lines.append(f"\nOverall RMS reprojection error: {rms:.4f} px")

    if rms < 0.5:
        quality = "EXCELLENT"
    elif rms < 1.0:
        quality = "GOOD"
    elif rms < 2.0:
        quality = "ACCEPTABLE"
    else:
        quality = "POOR: recapture recommended"
    lines.append(f"Quality assessment: {quality}")

    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    lines.append(f"\nCamera matrix:")
    lines.append(f"  fx={fx:.2f}  fy={fy:.2f}")
    lines.append(f"  cx={cx:.2f}  cy={cy:.2f}")
    lines.append(f"  aspect ratio fy/fx={fy/fx:.4f}") # should be around 1.0

    dist = dist_coeffs.flatten()
    _base_labels = ["k1", "k2", "p1", "p2", "k3", "k4", "k5", "k6"]
    dist_labels = _base_labels[:len(dist)]
    lines.append(f"\nDistortion coefficients ({' '.join(dist_labels)})  "
                 f"[{len(dist)} params]:")
    lines.append("  " + "  ".join(f"{lbl}={v:.6f}" for lbl, v in zip(dist_labels, dist)))

    lines.append(f"\nPer-image reprojection errors:")
    for path, err in sorted(zip(paths, per_errors), key=lambda x: x[1]):
        marker = " OK" if err < 1.0 else " !"
        lines.append(f"  {os.path.basename(path):40s}  {err:.4f} px{marker}")

    lines.append("\n" + "=" * 60)
    report = "\n".join(lines)
    print(report)

    report_path = os.path.join(out_dir, "calibration_report.txt")
    with open(report_path, "w") as f:
        f.write(report)
    print(f"\n% Report saved -> {report_path}")


def show_undistort_preview(camera_matrix, dist_coeffs, img_path):
    img = cv2.imread(img_path)
    if img is None:
        return
    h, w = img.shape[:2]
    # alpha = 1 keeps all pixels but introduces black borders; we crop them away
    new_mtx, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), alpha=1, newImgSize=(w, h))
    undist = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_mtx)
    rx, ry, rw, rh = roi
    if rw > 0 and rh > 0:
        cropped = undist[ry:ry + rh, rx:rx + rw]
        canvas = np.zeros_like(undist)  # black, same size as original
        canvas[:rh, :rw] = cropped      # top-left anchored, shows true crop extent
        undist = canvas

    for x in range(0, w, w // 10):
        cv2.line(img, (x, 0), (x, h), (0, 255, 0), 1)
        cv2.line(undist, (x, 0), (x, h), (0, 255, 0), 1)
    for y in range(0, h, h // 8):
        cv2.line(img, (0, y), (w, y), (0, 255, 0), 1)
        cv2.line(undist, (0, y), (w, y), (0, 255, 0), 1)

    cv2.putText(img, "ORIGINAL", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    cv2.putText(undist, "UNDISTORTED", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 200, 0), 2)

    side_by_side = np.hstack([img, undist])
    side_by_side = cv2.resize(side_by_side, (min(1600, side_by_side.shape[1]),
                                             min(500, side_by_side.shape[0])))
    cv2.imshow("Undistortion preview (any key to close)", side_by_side)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    parser = argparse.ArgumentParser(description="Calibrate camera from checkerboard images")
    parser.add_argument("--images", default="calibration/images",
                        help="Directory containing captured .jpg images")
    parser.add_argument("--out", default="calibration",
                        help="Output directory for .npz and report")
    parser.add_argument("--no-preview", action="store_true",
                        help="Skip undistortion preview window")
    args = parser.parse_args()

    os.makedirs(args.out, exist_ok=True)

    # Sanity check 
    if SQUARE_SIZE <= 0:
        raise ValueError("SQUARE_SIZE must be > 0")
    if SQUARE_SIZE > 0.20:
        raise ValueError(f"SQUARE_SIZE={SQUARE_SIZE} looks too large")

    board_size = (BOARD_W, BOARD_H)
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    print(f"% Board: {BOARD_W}×{BOARD_H}, square={SQUARE_SIZE*1000:.1f}mm")
    print("% Finding corners ...")
    objpoints, imgpoints, used_paths, img_size = find_corners(
        args.images, board_size, criteria)

    if len(objpoints) < 10:
        print(f"% ERROR: Only {len(objpoints)} usable images. Need at least 10.")
        print("%   Capture more images with better checkerboard visibility.")
        return

    print(f"\n% Running iterative calibration on {len(objpoints)} images ...")
    (camera_matrix, dist_coeffs, rvecs, tvecs,
     per_errors, final_paths, rms) = calibrate_iterative(
        objpoints, imgpoints, used_paths, img_size, criteria)
  
    # board_size and square_size are stored as numpy arrays so the .npz is
    npz_path = os.path.join(args.out, "camera_params.npz")
    np.savez(npz_path,
             camera_matrix=camera_matrix,
             dist_coeffs=dist_coeffs,
             rms=np.array(rms),
             img_size=np.array(img_size),
             board_size=np.array(board_size),
             square_size=np.array(SQUARE_SIZE))
    print(f"\n% Calibration saved -> {npz_path}")
    print(f"%   Load with: data = np.load('{npz_path}')")
    print(f"%              camera_matrix = data['camera_matrix']")
    print(f"%              dist_coeffs   = data['dist_coeffs']")

    quality_report(camera_matrix, dist_coeffs, rms, per_errors,
                   final_paths, img_size, args.out)

    # Undistortion preview
    if not args.no_preview and final_paths:
        print("\n% Showing undistortion preview ...")
        show_undistort_preview(camera_matrix, dist_coeffs, final_paths[0])


if __name__ == "__main__":
    main()