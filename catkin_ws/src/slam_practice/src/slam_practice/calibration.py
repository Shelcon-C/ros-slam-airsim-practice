"""相机标定结果的数据模型与 ORB-SLAM3 配置输出。"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple


@dataclass(frozen=True)
class CameraCalibration:
    width: int
    height: int
    fx: float
    fy: float
    cx: float
    cy: float
    distortion: Tuple[float, float, float, float]
    fps: float

    def __post_init__(self) -> None:
        if self.width <= 0 or self.height <= 0:
            raise ValueError("image dimensions must be positive")
        if self.fx <= 0 or self.fy <= 0:
            raise ValueError("focal lengths must be positive")
        if self.fps <= 0:
            raise ValueError("camera fps must be positive")
        if len(self.distortion) != 4:
            raise ValueError("distortion must contain k1, k2, p1, p2")
        numeric = (self.fx, self.fy, self.cx, self.cy, self.fps, *self.distortion)
        if not all(math.isfinite(float(value)) for value in numeric):
            raise ValueError("calibration values must be finite")


def orbslam_yaml_text(calibration: CameraCalibration) -> str:
    """生成可直接写入 ORB-SLAM3 设置文件的相机部分。"""

    k1, k2, p1, p2 = calibration.distortion
    return f'''%YAML:1.0
File.version: "1.0"
Camera.type: "PinHole"
Camera1.fx: {calibration.fx:.9f}
Camera1.fy: {calibration.fy:.9f}
Camera1.cx: {calibration.cx:.9f}
Camera1.cy: {calibration.cy:.9f}
Camera1.k1: {k1:.9f}
Camera1.k2: {k2:.9f}
Camera1.p1: {p1:.9f}
Camera1.p2: {p2:.9f}
Camera.width: {calibration.width}
Camera.height: {calibration.height}
Camera.fps: {calibration.fps:.9f}
Camera.RGB: 0

ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
'''
