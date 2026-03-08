import csv
from typing import List

import numpy as np


class CSVDataLoader:
    """Load Kalman test measurements and control inputs from CSV files."""

    def load_scalar_measurements(self, csv_path: str, measurement_column: str = "pos_x") -> List[float]:
        measurements: List[float] = []
        with open(csv_path, "r", newline="", encoding="utf-8") as f:
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
        with open(csv_path, "r", newline="", encoding="utf-8") as f:
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
        with open(csv_path, "r", newline="", encoding="utf-8") as f:
            reader = csv.DictReader(f)
            for row in reader:
                vector = np.array([[float(row[col])] for col in columns], dtype=float)
                measurements.append(vector)
        return measurements
