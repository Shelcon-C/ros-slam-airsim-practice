#!/usr/bin/env python3
"""运行 evo ATE/RPE，并保存图表、结果包、表格和命令日志。"""

import argparse
from pathlib import Path

from slam_practice.evaluation import build_evo_commands, run_evo_commands


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("groundtruth", type=Path)
    parser.add_argument("estimate", type=Path)
    parser.add_argument("output_dir", type=Path)
    parser.add_argument(
        "--sensor",
        choices=("monocular", "stereo-inertial"),
        default="monocular",
        help="单目会额外进行尺度校正",
    )
    args = parser.parse_args()
    for trajectory in (args.groundtruth, args.estimate):
        if not trajectory.expanduser().is_file():
            raise FileNotFoundError(trajectory)
    commands = build_evo_commands(
        args.groundtruth.expanduser().resolve(),
        args.estimate.expanduser().resolve(),
        args.output_dir,
        monocular=args.sensor == "monocular",
    )
    run_evo_commands(commands, args.output_dir)
    print(f"Evaluation artifacts written to {args.output_dir}")


if __name__ == "__main__":
    main()
