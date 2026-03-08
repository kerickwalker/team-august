import csv
from pathlib import Path
from typing import List

import numpy as np


class CSVDataLoader:
    """Load Kalman test measurements and control inputs from CSV files."""

    def _resolve_csv_path(self, csv_path: str) -> Path:
        """Resolve CSV path from current cwd or local csv/* folders."""
        requested = Path(csv_path)
        if requested.exists():
            return requested

        base_dir = Path(__file__).resolve().parent

        candidates = [
            base_dir / requested,
            base_dir / "csv" / requested.name,
            base_dir / "csv" / "controls" / requested.name,
            base_dir / "csv" / "measurements" / requested.name,
        ]

        for candidate in candidates:
            if candidate.exists():
                return candidate

        raise FileNotFoundError(
            f"Could not locate CSV file '{csv_path}'. Tried: "
            + ", ".join(str(path) for path in candidates)
        )

    def load_scalar_measurements(self, csv_path: str, measurement_column: str = "pos_x") -> List[float]:
        measurements: List[float] = []
        resolved_path = self._resolve_csv_path(csv_path)
        with open(resolved_path, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                measurements.append(float(row[measurement_column]))
        return measurements

    def load_control_inputs_2x1(
        self,
        csv_path: str,
        left_column: str = "u_left",
        right_column: str = "u_right",
    ) -> List[np.ndarray]:
        controls: List[np.ndarray] = []
        resolved_path = self._resolve_csv_path(csv_path)
        with open(resolved_path, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                controls.append(
                    np.array(
                        [[float(row[left_column])], [float(row[right_column])]],
                        dtype=float,
                    )
                )
        return controls

    def load_measurements_7x1(
        self,
        csv_path: str,
        columns: List[str],
    ) -> List[np.ndarray]:
        """Load multi-sensor measurements as 7x1 vectors (or Nx1 for len(columns))."""

        measurements: List[np.ndarray] = []
        resolved_path = self._resolve_csv_path(csv_path)
        with open(resolved_path, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                vector = np.array([[float(row[col])] for col in columns], dtype=float)
                measurements.append(vector)
        return measurements

    def load_timestamps(self, csv_path: str, time_column: str = "t_s") -> List[float]:
        """Load timestamp values from CSV for variable-step filtering."""

        timestamps: List[float] = []
        resolved_path = self._resolve_csv_path(csv_path)
        with open(resolved_path, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                timestamps.append(float(row[time_column]))
        return timestamps
