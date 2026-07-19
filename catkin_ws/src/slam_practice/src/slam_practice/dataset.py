"""EuRoC 数据索引读取。

该模块不依赖 ROS，便于先检查数据完整性，再启动计算量较大的 SLAM 节点。
"""

from __future__ import annotations

import csv
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List


class DatasetFormatError(ValueError):
    """数据清单格式不满足本项目约束。"""


@dataclass(frozen=True)
class Frame:
    """一帧带 EuRoC 纳秒时间戳的图像。"""

    timestamp_ns: int
    path: Path

    @property
    def timestamp_sec(self) -> float:
        return self.timestamp_ns * 1e-9

    @property
    def filename(self) -> str:
        return self.path.name


def _data_rows(index_path: Path) -> Iterable[list[str]]:
    with index_path.open("r", encoding="utf-8", newline="") as handle:
        for raw_line in handle:
            line = raw_line.strip()
            if not line or line.startswith("#"):
                continue
            row = next(csv.reader([line]))
            if len(row) < 2:
                raise DatasetFormatError(f"Malformed EuRoC row in {index_path}: {line}")
            yield row


def load_euroc_camera_index(root: Path | str, camera: str = "cam0") -> List[Frame]:
    """读取 ``mav0/<camera>/data.csv`` 并严格验证图像与时间戳。

    Args:
        root: EuRoC 序列根目录，例如 ``MH_01_easy``。
        camera: 相机目录名，任务一固定使用 ``cam0``。
    """

    root = Path(root).expanduser().resolve()
    camera_dir = root / "mav0" / camera
    index_path = camera_dir / "data.csv"
    image_dir = camera_dir / "data"
    if not index_path.is_file():
        raise FileNotFoundError(f"EuRoC camera index not found: {index_path}")

    frames: List[Frame] = []
    previous_timestamp = -1
    for row in _data_rows(index_path):
        try:
            timestamp_ns = int(row[0].strip())
        except ValueError as error:
            raise DatasetFormatError(f"Invalid timestamp in {index_path}: {row[0]}") from error
        if timestamp_ns <= previous_timestamp:
            raise DatasetFormatError(
                f"EuRoC timestamps must be strictly increasing: {timestamp_ns} after {previous_timestamp}"
            )
        image_path = image_dir / row[1].strip()
        if not image_path.is_file():
            raise FileNotFoundError(f"EuRoC image not found: {image_path}")
        frames.append(Frame(timestamp_ns=timestamp_ns, path=image_path))
        previous_timestamp = timestamp_ns

    if not frames:
        raise DatasetFormatError(f"No frames found in {index_path}")
    return frames
