#!/usr/bin/env python3
"""将 EuRoC 真值 CSV 转换为 evo 可直接读取的 TUM 格式。"""

import argparse
from pathlib import Path

import cv2

from slam_practice.trajectory import convert_euroc_groundtruth


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("source", help="mav0/state_groundtruth_estimate0/data.csv")
    parser.add_argument("destination", help="输出的 groundtruth.tum")
    parser.add_argument(
        "--sensor-yaml",
        type=Path,
        help="EuRoC cam0/sensor.yaml；读取 T_BS，把 IMU/body 真值转换到相机光学系",
    )
    args = parser.parse_args()
    body_T_sensor = None
    if args.sensor_yaml is not None:
        sensor_yaml = args.sensor_yaml.expanduser().resolve()
        if not sensor_yaml.is_file():
            raise FileNotFoundError(sensor_yaml)
        storage = cv2.FileStorage(str(sensor_yaml), cv2.FILE_STORAGE_READ)
        try:
            body_T_sensor = storage.getNode("T_BS").mat()
        finally:
            storage.release()
        if body_T_sensor is None or body_T_sensor.shape != (4, 4):
            raise ValueError(f"T_BS 4x4 matrix not found in {sensor_yaml}")
    count = convert_euroc_groundtruth(
        args.source, args.destination, body_T_sensor=body_T_sensor
    )
    print(f"Converted {count} ground-truth poses to {args.destination}")


if __name__ == "__main__":
    main()
