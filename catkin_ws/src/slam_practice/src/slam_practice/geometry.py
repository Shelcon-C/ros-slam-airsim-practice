"""AirSim、ROS 与相机坐标系之间使用的几何纯函数。

四元数统一采用 ROS 的 ``(x, y, z, w)`` 顺序。
"""

from __future__ import annotations

import numpy as np


# 每一列表示相机光学系一个轴在 AirSim 机体系中的方向：
# optical x=body y，optical y=body z，optical z=body x。
AIRSIM_OPTICAL_TO_BODY = np.array(
    [[0.0, 0.0, 1.0], [1.0, 0.0, 0.0], [0.0, 1.0, 0.0]], dtype=float
)

# ROS REP-103 机体系为 FLU（x前/y左/z上），光学系仍为 x右/y下/z前。
ROS_OPTICAL_TO_BODY = np.array(
    [[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]], dtype=float
)


def _vector(values, size: int, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        raise ValueError(f"{name} must contain {size} finite values")
    return array


def ned_to_enu_vector(vector) -> np.ndarray:
    """NED ``(north,east,down)`` 转 ENU ``(east,north,up)``。"""

    north, east, down = _vector(vector, 3, "NED vector")
    return np.array((east, north, -down), dtype=float)


def normalize_quaternion(quaternion_xyzw) -> np.ndarray:
    quaternion = _vector(quaternion_xyzw, 4, "quaternion")
    norm = np.linalg.norm(quaternion)
    if norm < 1e-12:
        raise ValueError("quaternion norm is zero")
    return quaternion / norm


def quaternion_multiply(left_xyzw, right_xyzw) -> np.ndarray:
    x1, y1, z1, w1 = normalize_quaternion(left_xyzw)
    x2, y2, z2, w2 = normalize_quaternion(right_xyzw)
    product = np.array(
        (
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        ),
        dtype=float,
    )
    return normalize_quaternion(product)


def quaternion_to_matrix(quaternion_xyzw) -> np.ndarray:
    x, y, z, w = normalize_quaternion(quaternion_xyzw)
    return np.array(
        (
            (1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
            (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
            (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)),
        ),
        dtype=float,
    )


def matrix_to_quaternion(matrix) -> np.ndarray:
    """把正交旋转矩阵转换为 xyzw 四元数。"""

    rotation = np.asarray(matrix, dtype=float)
    if rotation.shape != (3, 3) or not np.all(np.isfinite(rotation)):
        raise ValueError("rotation matrix must be finite 3x3")
    trace = float(np.trace(rotation))
    if trace > 0:
        scale = np.sqrt(trace + 1.0) * 2
        quaternion = (
            (rotation[2, 1] - rotation[1, 2]) / scale,
            (rotation[0, 2] - rotation[2, 0]) / scale,
            (rotation[1, 0] - rotation[0, 1]) / scale,
            0.25 * scale,
        )
    else:
        axis = int(np.argmax(np.diag(rotation)))
        if axis == 0:
            scale = np.sqrt(1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            quaternion = (0.25 * scale, (rotation[0, 1] + rotation[1, 0]) / scale,
                          (rotation[0, 2] + rotation[2, 0]) / scale,
                          (rotation[2, 1] - rotation[1, 2]) / scale)
        elif axis == 1:
            scale = np.sqrt(1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            quaternion = ((rotation[0, 1] + rotation[1, 0]) / scale, 0.25 * scale,
                          (rotation[1, 2] + rotation[2, 1]) / scale,
                          (rotation[0, 2] - rotation[2, 0]) / scale)
        else:
            scale = np.sqrt(1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            quaternion = ((rotation[0, 2] + rotation[2, 0]) / scale,
                          (rotation[1, 2] + rotation[2, 1]) / scale, 0.25 * scale,
                          (rotation[1, 0] - rotation[0, 1]) / scale)
    return normalize_quaternion(quaternion)


AIRSIM_OPTICAL_TO_BODY_QUATERNION = matrix_to_quaternion(AIRSIM_OPTICAL_TO_BODY)
ROS_OPTICAL_TO_BODY_QUATERNION = matrix_to_quaternion(ROS_OPTICAL_TO_BODY)


def compose_pose(parent_position, parent_quaternion, child_position, child_quaternion):
    """计算 ``T_world_parent * T_parent_child``。"""

    parent_position = _vector(parent_position, 3, "parent position")
    child_position = _vector(child_position, 3, "child position")
    parent_rotation = quaternion_to_matrix(parent_quaternion)
    position = parent_position + parent_rotation @ child_position
    quaternion = quaternion_multiply(parent_quaternion, child_quaternion)
    return position, quaternion
