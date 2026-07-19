"""evo 轨迹评估命令构造与执行。"""

from __future__ import annotations

import subprocess
from pathlib import Path
from typing import List, Sequence


def build_evo_commands(
    groundtruth: Path | str,
    estimate: Path | str,
    output_dir: Path | str,
    monocular: bool,
) -> List[List[str]]:
    """构造 ATE、RPE 与汇总命令。

    返回参数数组而不是字符串，调用端可以安全地传给 ``subprocess.run``。
    """

    groundtruth = Path(groundtruth)
    estimate = Path(estimate)
    output_dir = Path(output_dir)
    alignment = ["--align"] + (["--correct_scale"] if monocular else [])
    common = ["tum", str(groundtruth), str(estimate), *alignment, "--plot_mode", "xyz"]
    ate_zip = output_dir / "ate_results.zip"
    rpe_translation_zip = output_dir / "rpe_translation_results.zip"
    rpe_rotation_zip = output_dir / "rpe_rotation_results.zip"
    return [
        ["evo_ape", *common, "--pose_relation", "trans_part", "--save_results", str(ate_zip),
         "--save_plot", str(output_dir / "ate_plot.pdf")],
        ["evo_rpe", *common, "--pose_relation", "trans_part", "--delta", "1", "--delta_unit", "f",
         "--save_results", str(rpe_translation_zip),
         "--save_plot", str(output_dir / "rpe_translation_plot.pdf")],
        ["evo_rpe", *common, "--pose_relation", "angle_deg", "--delta", "1", "--delta_unit", "f",
         "--save_results", str(rpe_rotation_zip),
         "--save_plot", str(output_dir / "rpe_rotation_plot.pdf")],
        ["evo_res", str(ate_zip), str(rpe_translation_zip), str(rpe_rotation_zip), "--use_filenames",
         "--save_table", str(output_dir / "metrics.csv")],
    ]


def run_evo_commands(commands: Sequence[Sequence[str]], output_dir: Path | str) -> None:
    """顺序执行 evo 命令，并把标准输出保存为可追溯日志。"""

    output_dir = Path(output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    for index, command in enumerate(commands, start=1):
        completed = subprocess.run(
            list(command),
            check=True,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
        )
        (output_dir / f"command_{index}.log").write_text(completed.stdout, encoding="utf-8")
