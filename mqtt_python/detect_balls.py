#!/usr/bin/env python3

import argparse
import json
import os
from dataclasses import dataclass

import cv2 as cv
import numpy as np


WINDOW_FRAME = "Ball detection"
WINDOW_MASK = "Active colour mask"
WINDOW_CTRL = "Tuning controls"

COLOUR_KEYS = {
	"b": "blue",
	"r": "red",
	"w": "white",
}

DRAW_COLOURS = {
	"blue": (255, 100, 20),
	"red": (40, 40, 255),
	"white": (220, 220, 220),
}


@dataclass
class ColourThreshold:
	h_low: int
	h_high: int
	s_low: int
	s_high: int
	v_low: int
	v_high: int


def default_config():
	return {
		"blue": ColourThreshold(90, 135, 70, 255, 50, 255),
		# Red wraps hue around 0 in HSV, so h_low > h_high is intentional.
		"red": ColourThreshold(170, 10, 80, 255, 60, 255),
		"white": ColourThreshold(0, 179, 0, 70, 150, 255),
		"detector": {
			"min_radius": 8,
			"max_radius": 120,
			"min_area": 200,
			"min_circularity_x100": 70,
			"open_kernel": 3,
			"close_kernel": 5,
		},
	}


def create_trackbars(config, active_colour):
	cv.namedWindow(WINDOW_CTRL, cv.WINDOW_NORMAL)
	cv.resizeWindow(WINDOW_CTRL, 480, 360)

	def _noop(_):
		return

	ct = config[active_colour]
	det = config["detector"]

	cv.createTrackbar("H low", WINDOW_CTRL, ct.h_low, 179, _noop)
	cv.createTrackbar("H high", WINDOW_CTRL, ct.h_high, 179, _noop)
	cv.createTrackbar("S low", WINDOW_CTRL, ct.s_low, 255, _noop)
	cv.createTrackbar("S high", WINDOW_CTRL, ct.s_high, 255, _noop)
	cv.createTrackbar("V low", WINDOW_CTRL, ct.v_low, 255, _noop)
	cv.createTrackbar("V high", WINDOW_CTRL, ct.v_high, 255, _noop)

	cv.createTrackbar("Min radius", WINDOW_CTRL, det["min_radius"], 200, _noop)
	cv.createTrackbar("Max radius", WINDOW_CTRL, det["max_radius"], 250, _noop)
	cv.createTrackbar("Min area", WINDOW_CTRL, det["min_area"], 20000, _noop)
	cv.createTrackbar(
		"Min circularity x100",
		WINDOW_CTRL,
		det["min_circularity_x100"],
		100,
		_noop,
	)
	cv.createTrackbar("Open kernel", WINDOW_CTRL, det["open_kernel"], 15, _noop)
	cv.createTrackbar("Close kernel", WINDOW_CTRL, det["close_kernel"], 15, _noop)


def write_trackbars_to_config(config, active_colour):
	ct = config[active_colour]
	det = config["detector"]

	ct.h_low = cv.getTrackbarPos("H low", WINDOW_CTRL)
	ct.h_high = cv.getTrackbarPos("H high", WINDOW_CTRL)
	ct.s_low = cv.getTrackbarPos("S low", WINDOW_CTRL)
	ct.s_high = cv.getTrackbarPos("S high", WINDOW_CTRL)
	ct.v_low = cv.getTrackbarPos("V low", WINDOW_CTRL)
	ct.v_high = cv.getTrackbarPos("V high", WINDOW_CTRL)

	det["min_radius"] = max(1, cv.getTrackbarPos("Min radius", WINDOW_CTRL))
	det["max_radius"] = max(det["min_radius"], cv.getTrackbarPos("Max radius", WINDOW_CTRL))
	det["min_area"] = max(10, cv.getTrackbarPos("Min area", WINDOW_CTRL))
	det["min_circularity_x100"] = cv.getTrackbarPos("Min circularity x100", WINDOW_CTRL)
	det["open_kernel"] = cv.getTrackbarPos("Open kernel", WINDOW_CTRL)
	det["close_kernel"] = cv.getTrackbarPos("Close kernel", WINDOW_CTRL)


def set_trackbars_from_colour(config, active_colour):
	ct = config[active_colour]
	cv.setTrackbarPos("H low", WINDOW_CTRL, int(ct.h_low))
	cv.setTrackbarPos("H high", WINDOW_CTRL, int(ct.h_high))
	cv.setTrackbarPos("S low", WINDOW_CTRL, int(ct.s_low))
	cv.setTrackbarPos("S high", WINDOW_CTRL, int(ct.s_high))
	cv.setTrackbarPos("V low", WINDOW_CTRL, int(ct.v_low))
	cv.setTrackbarPos("V high", WINDOW_CTRL, int(ct.v_high))


def make_mask(hsv_img, threshold: ColourThreshold):
	lower = np.array([threshold.h_low, threshold.s_low, threshold.v_low], dtype=np.uint8)
	upper = np.array([threshold.h_high, threshold.s_high, threshold.v_high], dtype=np.uint8)

	if threshold.h_low <= threshold.h_high:
		return cv.inRange(hsv_img, lower, upper)

	# Hue range wraps around 179->0 for colours near red.
	lower_1 = np.array([0, threshold.s_low, threshold.v_low], dtype=np.uint8)
	upper_1 = np.array([threshold.h_high, threshold.s_high, threshold.v_high], dtype=np.uint8)
	lower_2 = np.array([threshold.h_low, threshold.s_low, threshold.v_low], dtype=np.uint8)
	upper_2 = np.array([179, threshold.s_high, threshold.v_high], dtype=np.uint8)
	return cv.bitwise_or(cv.inRange(hsv_img, lower_1, upper_1), cv.inRange(hsv_img, lower_2, upper_2))


def apply_morphology(mask, open_kernel, close_kernel):
	if open_kernel > 1:
		k = cv.getStructuringElement(cv.MORPH_ELLIPSE, (open_kernel, open_kernel))
		mask = cv.morphologyEx(mask, cv.MORPH_OPEN, k, iterations=1)
	if close_kernel > 1:
		k = cv.getStructuringElement(cv.MORPH_ELLIPSE, (close_kernel, close_kernel))
		mask = cv.morphologyEx(mask, cv.MORPH_CLOSE, k, iterations=1)
	return mask


def detect_balls_in_mask(mask, det_cfg, colour_name):
	contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
	detections = []

	min_area = float(det_cfg["min_area"])
	min_radius = float(det_cfg["min_radius"])
	max_radius = float(det_cfg["max_radius"])
	min_circularity = float(det_cfg["min_circularity_x100"]) / 100.0

	for contour in contours:
		area = cv.contourArea(contour)
		if area < min_area:
			continue

		perimeter = cv.arcLength(contour, True)
		if perimeter <= 0:
			continue

		circularity = 4.0 * np.pi * area / (perimeter * perimeter)
		if circularity < min_circularity:
			continue

		(x, y), radius = cv.minEnclosingCircle(contour)
		if radius < min_radius or radius > max_radius:
			continue

		detections.append(
			{
				"colour": colour_name,
				"center": (int(x), int(y)),
				"radius": int(radius),
				"area": float(area),
				"circularity": float(circularity),
			}
		)

	return detections


def load_config(path):
	cfg = default_config()
	if not path or not os.path.exists(path):
		return cfg

	with open(path, "r", encoding="utf-8") as f:
		raw = json.load(f)

	for colour in ("blue", "red", "white"):
		if colour in raw:
			item = raw[colour]
			cfg[colour] = ColourThreshold(
				int(item["h_low"]),
				int(item["h_high"]),
				int(item["s_low"]),
				int(item["s_high"]),
				int(item["v_low"]),
				int(item["v_high"]),
			)

	if "detector" in raw:
		cfg["detector"].update(raw["detector"])

	return cfg


def save_config(config, path):
	if not path:
		return

	out = {
		"blue": vars(config["blue"]),
		"red": vars(config["red"]),
		"white": vars(config["white"]),
		"detector": config["detector"],
	}
	with open(path, "w", encoding="utf-8") as f:
		json.dump(out, f, indent=2)
	print(f"Saved configuration to {path}")


def parse_args():
	parser = argparse.ArgumentParser(description="Detect blue/red/white balls and show center locations.")
	parser.add_argument(
		"--source",
		default="0",
		help="Camera source index or URL (default: 0).",
	)
	parser.add_argument(
		"--config",
		default="ball_detect_config.json",
		help="Path to load/save tuning configuration.",
	)
	parser.add_argument(
		"--no-gui",
		action="store_true",
		help="Run without OpenCV windows and print detections only.",
	)
	return parser.parse_args()


def open_capture(source_arg):
	if source_arg.isdigit():
		return cv.VideoCapture(int(source_arg))
	return cv.VideoCapture(source_arg)


def main():
	args = parse_args()
	config = load_config(args.config)

	cap = open_capture(args.source)
	if not cap.isOpened():
		print(f"Failed to open camera source: {args.source}")
		return

	active_colour = "blue"

	if not args.no_gui:
		cv.namedWindow(WINDOW_FRAME, cv.WINDOW_NORMAL)
		cv.namedWindow(WINDOW_MASK, cv.WINDOW_NORMAL)
		create_trackbars(config, active_colour)
		print("Controls: b/r/w switch colour tuning, s save config, q quit")

	while True:
		ret, frame = cap.read()
		if not ret:
			print("Camera frame read failed.")
			break

		if not args.no_gui:
			write_trackbars_to_config(config, active_colour)

		hsv = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
		all_detections = []
		active_mask = None

		for colour_name in ("blue", "red", "white"):
			raw_mask = make_mask(hsv, config[colour_name])
			mask = apply_morphology(
				raw_mask,
				config["detector"]["open_kernel"],
				config["detector"]["close_kernel"],
			)

			if colour_name == active_colour:
				active_mask = mask

			all_detections.extend(detect_balls_in_mask(mask, config["detector"], colour_name))

		for det in all_detections:
			x, y = det["center"]
			radius = det["radius"]
			colour = det["colour"]
			draw_col = DRAW_COLOURS[colour]

			cv.circle(frame, (x, y), radius, draw_col, 2)
			cv.circle(frame, (x, y), 3, (0, 255, 0), -1)
			cv.putText(
				frame,
				f"{colour} ({x},{y})",
				(x + 8, y - 8),
				cv.FONT_HERSHEY_SIMPLEX,
				0.55,
				draw_col,
				2,
			)

		if all_detections:
			msg = "; ".join(
				[f"{d['colour']}@({d['center'][0]},{d['center'][1]}) r={d['radius']}" for d in all_detections]
			)
			print(msg)

		if args.no_gui:
			continue

		cv.putText(
			frame,
			f"Tuning: {active_colour} | balls: {len(all_detections)}",
			(10, 30),
			cv.FONT_HERSHEY_SIMPLEX,
			0.8,
			(20, 255, 20),
			2,
		)

		cv.imshow(WINDOW_FRAME, frame)
		if active_mask is not None:
			cv.imshow(WINDOW_MASK, active_mask)

		key = cv.waitKey(1) & 0xFF
		if key == ord("q"):
			break
		if key == ord("s"):
			save_config(config, args.config)
		if chr(key) in COLOUR_KEYS:
			active_colour = COLOUR_KEYS[chr(key)]
			set_trackbars_from_colour(config, active_colour)

	cap.release()
	cv.destroyAllWindows()


if __name__ == "__main__":
	main()
