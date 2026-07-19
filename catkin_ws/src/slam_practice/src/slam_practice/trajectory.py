"""TUM 轨迹格式化与 EuRoC 真值转换。"""

from __future__ import annotations

import csv
import math
from pathlib import Path
from typing import Iterable, Sequence

import numpy as np

from slam_practice.geometry import compose_pose, matrix_to_quaternion


def _finite(values: Iterable[float]) -> bool:
    return all(math.isfinite(float(value)) for value in values)


def format_tum_pose(
    timestamp_sec: float,
    position_xyz: Sequence[float],
    quaternion_xyzw: Sequence[float],
) -> str:
    """返回 ``timestamp tx ty tz qx qy qz qw`` 格式的一行。"""

    if len(position_xyz) != 3 or len(quaternion_xyzw) != 4:
        raise ValueError("TUM pose requires three position and four quaternion values")
    values = (timestamp_sec, *position_xyz, *quaternion_xyzw)
    if not _finite(values):
        raise ValueError("TUM pose values must be finite")
    return " ".join(f"{float(value):.9f}" for value in values)


def convert_euroc_groundtruth(
    source: Path | str,
    destination: Path | str,
    body_T_sensor=None,
) -> int:
    """把 EuRoC 真值 CSV 转为 TUM，并返回写入的位姿数量。

    EuRoC 四元数顺序是 ``qw,qx,qy,qz``，TUM 要求 ``qx,qy,qz,qw``。
    """

    source = Path(source).expanduser().resolve()
    destination = Path(destination).expanduser().resolve()
    if not source.is_file():
        raise FileNotFoundError(f"EuRoC ground truth not found: {source}")

    sensor_translation = None
    sensor_quaternion = None
    if body_T_sensor is not None:
        transform = np.asarray(body_T_sensor, dtype=float)
        if transform.shape != (4, 4) or not np.all(np.isfinite(transform)):
            raise ValueError("body_T_sensor must be a finite 4x4 matrix")
        if not np.allclose(transform[3], (0.0, 0.0, 0.0, 1.0), atol=1e-9):
            raise ValueError("body_T_sensor must be a homogeneous transform")
        sensor_translation = transform[:3, 3]
        sensor_quaternion = matrix_to_quaternion(transform[:3, :3])

    lines = []
    with source.open("r", encoding="utf-8", newline="") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            row = next(csv.reader([line]))
            if len(row) < 8:
                raise ValueError(f"Malformed EuRoC ground-truth row: {line}")
            timestamp_sec = int(row[0].strip()) * 1e-9
            position = tuple(float(value) for value in row[1:4])
            qw, qx, qy, qz = (float(value) for value in row[4:8])
            quaternion = (qx, qy, qz, qw)
            if sensor_translation is not None:
                position, quaternion = compose_pose(
                    position, quaternion, sensor_translation, sensor_quaternion
                )
            lines.append(format_tum_pose(timestamp_sec, position, quaternion))

    if not lines:
        raise ValueError(f"No ground-truth poses found in {source}")
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return len(lines)
