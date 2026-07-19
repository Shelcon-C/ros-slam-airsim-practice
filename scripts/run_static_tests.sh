#!/usr/bin/env bash
set -euo pipefail

REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PACKAGE_ROOT="${REPOSITORY_ROOT}/catkin_ws/src/slam_practice"
export PYTHONPATH="${PACKAGE_ROOT}/src${PYTHONPATH:+:${PYTHONPATH}}"

python3 -m unittest discover -s "${PACKAGE_ROOT}/tests" -p 'test_*.py' -v
python3 -m compileall -q "${PACKAGE_ROOT}/src" "${PACKAGE_ROOT}/scripts"
bash -n "${REPOSITORY_ROOT}"/scripts/*.sh

echo "静态测试、Python 语法和 Shell 语法检查通过。"
