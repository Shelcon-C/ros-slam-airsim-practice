"""不依赖 ROS 的无人机位置—速度控制纯函数。"""

from __future__ import annotations

import numpy as np


def _finite_vector(values, name: str) -> np.ndarray:
    vector = np.asarray(values, dtype=float)
    if vector.shape != (3,) or not np.all(np.isfinite(vector)):
        raise ValueError(f"{name} must contain three finite values")
    return vector


def compute_velocity_command(current, target, feedforward, kp, max_xy, max_z) -> np.ndarray:
    """计算带前馈的 P 控制速度，并分别限制水平模长和垂直速度。"""

    current_vector = _finite_vector(current, "current position")
    target_vector = _finite_vector(target, "target position")
    feedforward_vector = _finite_vector(feedforward, "feedforward velocity")
    gains = np.asarray((kp, max_xy, max_z), dtype=float)
    if not np.all(np.isfinite(gains)) or kp < 0 or max_xy <= 0 or max_z <= 0:
        raise ValueError("kp must be non-negative and velocity limits must be positive")

    velocity = feedforward_vector + float(kp) * (target_vector - current_vector)
    horizontal_norm = float(np.linalg.norm(velocity[:2]))
    if horizontal_norm > max_xy:
        velocity[:2] *= float(max_xy) / horizontal_norm
    velocity[2] = np.clip(velocity[2], -float(max_z), float(max_z))
    return velocity
