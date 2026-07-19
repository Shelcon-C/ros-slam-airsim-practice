#!/usr/bin/env python3
"""使用棋盘格图像或视频标定针孔相机并输出 ORB-SLAM3 YAML。"""

from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np

from slam_practice.calibration import CameraCalibration, orbslam_yaml_text


def collect_images(image_glob: str | None, video: str | None, stride: int):
    if image_glob:
        paths = sorted(glob.glob(image_glob))
        if not paths:
            raise FileNotFoundError(f"No calibration images match: {image_glob}")
        for path in paths:
            image = cv2.imread(path, cv2.IMREAD_COLOR)
            if image is not None:
                yield path, image
        return

    capture = cv2.VideoCapture(str(video))
    if not capture.isOpened():
        raise FileNotFoundError(f"Unable to open calibration video: {video}")
    index = 0
    while True:
        ok, image = capture.read()
        if not ok:
            break
        if index % stride == 0:
            yield f"frame_{index:06d}", image
        index += 1
    capture.release()


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--images", help="棋盘格图片通配符，例如 'calib/*.png'")
    source.add_argument("--video", help="棋盘格标定视频")
    parser.add_argument("--board-cols", type=int, required=True, help="棋盘格横向内角点数量")
    parser.add_argument("--board-rows", type=int, required=True, help="棋盘格纵向内角点数量")
    parser.add_argument("--square-size", type=float, required=True, help="格子边长，单位米")
    parser.add_argument("--fps", type=float, required=True)
    parser.add_argument("--stride", type=int, default=10, help="视频每隔多少帧取一帧")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--report", type=Path, required=True)
    args = parser.parse_args()
    if args.board_cols <= 1 or args.board_rows <= 1 or args.square_size <= 0 or args.stride <= 0:
        raise ValueError("board dimensions, square size and stride must be positive")

    pattern_size = (args.board_cols, args.board_rows)
    object_template = np.zeros((args.board_cols * args.board_rows, 3), np.float32)
    object_template[:, :2] = np.mgrid[0:args.board_cols, 0:args.board_rows].T.reshape(-1, 2)
    object_template *= args.square_size
    object_points = []
    image_points = []
    accepted = []
    image_size = None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-3)

    for name, image in collect_images(args.images, args.video, args.stride):
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        image_size = (gray.shape[1], gray.shape[0])
        found, corners = cv2.findChessboardCorners(gray, pattern_size)
        if not found:
            continue
        refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        object_points.append(object_template.copy())
        image_points.append(refined)
        accepted.append(name)

    if image_size is None or len(accepted) < 10:
        raise RuntimeError(f"At least 10 valid checkerboard views are required; found {len(accepted)}")

    rms, matrix, distortion, rotations, translations = cv2.calibrateCamera(
        object_points, image_points, image_size, None, None
    )
    errors = []
    for object_point, detected, rotation, translation in zip(
        object_points, image_points, rotations, translations
    ):
        projected, _ = cv2.projectPoints(object_point, rotation, translation, matrix, distortion)
        errors.append(float(cv2.norm(detected, projected, cv2.NORM_L2) / len(projected)))

    coefficients = distortion.reshape(-1)
    padded = np.pad(coefficients, (0, max(0, 4 - len(coefficients))))
    calibration = CameraCalibration(
        width=image_size[0],
        height=image_size[1],
        fx=float(matrix[0, 0]),
        fy=float(matrix[1, 1]),
        cx=float(matrix[0, 2]),
        cy=float(matrix[1, 2]),
        distortion=tuple(float(value) for value in padded[:4]),
        fps=args.fps,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(orbslam_yaml_text(calibration), encoding="utf-8")
    report = {
        "rms": float(rms),
        "mean_reprojection_error_px": float(np.mean(errors)),
        "accepted_view_count": len(accepted),
        "accepted_views": accepted,
        "image_size": list(image_size),
    }
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps(report, indent=2, ensure_ascii=False))


if __name__ == "__main__":
    main()
