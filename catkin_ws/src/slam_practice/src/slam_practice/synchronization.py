"""双目时间戳同步的纯函数。"""

import math


def validate_stereo_delta(left_stamp: float, right_stamp: float, tolerance: float = 0.003) -> float:
    """返回左右目时间差，超出容差时抛出异常。"""

    if not math.isfinite(left_stamp) or not math.isfinite(right_stamp):
        raise ValueError("stereo timestamps must be finite")
    if tolerance <= 0 or not math.isfinite(tolerance):
        raise ValueError("stereo tolerance must be finite and positive")
    delta = abs(left_stamp - right_stamp)
    if delta > tolerance:
        raise ValueError(
            f"stereo timestamp delta {delta * 1000:.3f} ms exceeds {tolerance * 1000:.3f} ms"
        )
    return delta


def synchronized_stamp(left_stamp: float, right_stamp: float, tolerance: float = 0.003) -> float:
    """验证时间差后返回二者平均时间，供左右图像共用。"""

    validate_stereo_delta(left_stamp, right_stamp, tolerance)
    return (left_stamp + right_stamp) * 0.5
