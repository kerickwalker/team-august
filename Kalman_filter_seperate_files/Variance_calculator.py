#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
from pathlib import Path
from statistics import pvariance, variance
from typing import Dict, List, Sequence


def find_csv_files(base_dir: Path) -> List[Path]:
	"""Return CSV files in the current folder and known csv subfolders."""
	search_roots = [
		base_dir,
		base_dir / "csv",
		base_dir / "csv" / "controls",
		base_dir / "csv" / "measurements",
	]

	discovered: List[Path] = []
	seen = set()
	for root in search_roots:
		if not root.exists() or not root.is_dir():
			continue
		for csv_file in sorted(root.glob("*.csv")):
			key = str(csv_file.resolve())
			if key not in seen:
				seen.add(key)
				discovered.append(csv_file)
	return discovered


def load_numeric_columns(csv_path: Path) -> Dict[str, List[float]]:
	"""Load numeric values per column from CSV, skipping non-numeric entries."""
	with open(csv_path, "r", newline="", encoding="utf-8") as f:
		reader = csv.DictReader(f)
		if not reader.fieldnames:
			raise ValueError("CSV has no header row")

		data: Dict[str, List[float]] = {name: [] for name in reader.fieldnames}

		for row in reader:
			for col in reader.fieldnames:
				value = row.get(col, "")
				if value is None:
					continue
				value = value.strip()
				if value == "":
					continue
				try:
					data[col].append(float(value))
				except ValueError:
					# Ignore non-numeric cells in a column.
					continue
	return data


def parse_column_selection(raw: str, columns: Sequence[str]) -> List[str]:
	"""Parse user selection supporting names, indices, and 'all'."""
	cleaned = raw.strip()
	if cleaned.lower() == "all":
		return list(columns)

	selected: List[str] = []
	tokens = [token.strip() for token in cleaned.split(",") if token.strip()]

	for token in tokens:
		if token.isdigit():
			idx = int(token)
			if idx < 1 or idx > len(columns):
				raise ValueError(f"Column index {idx} is out of range")
			name = columns[idx - 1]
		else:
			if token not in columns:
				raise ValueError(f"Column '{token}' not found")
			name = token

		if name not in selected:
			selected.append(name)

	if not selected:
		raise ValueError("No columns selected")

	return selected


def choose_csv_interactively(csv_files: Sequence[Path], base_dir: Path) -> Path:
	"""Prompt user to choose a CSV file."""
	print("Available CSV files:")
	for i, path in enumerate(csv_files, start=1):
		try:
			display = path.resolve().relative_to(base_dir.resolve())
		except ValueError:
			display = path.resolve()
		print(f"  {i}. {display}")

	choice = input("Select CSV file by number: ").strip()
	if not choice.isdigit():
		raise ValueError("File selection must be a number")

	idx = int(choice)
	if idx < 1 or idx > len(csv_files):
		raise ValueError("Selected file index is out of range")

	return csv_files[idx - 1]


def resolve_csv_path(input_path: str | None, base_dir: Path) -> Path:
	"""Resolve a CSV path from explicit input or interactive discovery."""
	if input_path:
		candidate = Path(input_path)
		if candidate.exists():
			return candidate

		fallback = base_dir / input_path
		if fallback.exists():
			return fallback

		raise FileNotFoundError(f"CSV file not found: {input_path}")

	csv_files = find_csv_files(base_dir)
	if not csv_files:
		raise FileNotFoundError("No CSV files found in current folder or csv subfolders")
	if len(csv_files) == 1:
		return csv_files[0]
	return choose_csv_interactively(csv_files, base_dir)


def main() -> int:
	parser = argparse.ArgumentParser(
		description="Read CSV and calculate variance for selected numeric columns.",
	)
	parser.add_argument(
		"--file",
		help="Path to CSV file. If omitted, script will prompt selection.",
	)
	parser.add_argument(
		"--columns",
		nargs="+",
		help="Column names to compute variance for. If omitted, script will prompt selection.",
	)
	mode_group = parser.add_mutually_exclusive_group()
	mode_group.add_argument(
		"--sample",
		action="store_true",
		help="Use sample variance (n-1).",
	)
	mode_group.add_argument(
		"--population",
		action="store_true",
		help="Use population variance (n).",
	)
	args = parser.parse_args()

	base_dir = Path(__file__).resolve().parent

	try:
		csv_path = resolve_csv_path(args.file, base_dir)
		data = load_numeric_columns(csv_path)
	except Exception as exc:
		print(f"Error: {exc}")
		return 1

	columns = list(data.keys())
	print(f"\nLoaded: {csv_path}")
	print("Columns:")
	for i, col in enumerate(columns, start=1):
		print(f"  {i}. {col}")

	try:
		if args.columns:
			selected_columns: List[str] = []
			for name in args.columns:
				if name not in data:
					raise ValueError(f"Column '{name}' not found")
				if name not in selected_columns:
					selected_columns.append(name)
		else:
			raw = input("\nChoose columns by names or indices (comma-separated), or type 'all': ")
			selected_columns = parse_column_selection(raw, columns)
	except Exception as exc:
		print(f"Selection error: {exc}")
		return 1

	if args.sample:
		use_sample = True
	elif args.population:
		use_sample = False
	else:
		choice = input("\nVariance type? [p]opulation / [s]ample (default: p): ").strip().lower()
		use_sample = choice in {"s", "sample"}

	print("\nVariance results:")
	for col in selected_columns:
		values = data[col]
		if len(values) == 0:
			print(f"- {col}: no numeric values found")
			continue
		if use_sample and len(values) < 2:
			print(f"- {col}: need at least 2 numeric values for sample variance")
			continue

		value = variance(values) if use_sample else pvariance(values)
		mode = "sample" if use_sample else "population"
		print(f"- {col} ({mode}, n={len(values)}): {value}")

	return 0


if __name__ == "__main__":
	raise SystemExit(main())
