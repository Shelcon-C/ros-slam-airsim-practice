#!/usr/bin/env bash
set -euo pipefail

REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY_ROOT="${THIRD_PARTY_ROOT:-${REPOSITORY_ROOT}/third_party}"
WORKSPACE="${REPOSITORY_ROOT}/catkin_ws"
EGO_ROOT="${THIRD_PARTY_ROOT}/ego-planner"
ORB_SLAM3_ROOT="${THIRD_PARTY_ROOT}/ORB_SLAM3"

if [[ "${ROS_DISTRO:-}" != "noetic" ]]; then
  echo "请先执行：source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi
if [[ ! -d "${EGO_ROOT}" ]]; then
  echo "未找到 ego-planner，请先运行 scripts/fetch_third_party.sh。" >&2
  exit 1
fi

if [[ ! -f "${THIRD_PARTY_ROOT}/AirSim/ros/devel/setup.bash" ]] || [[ ! -e "${WORKSPACE}/src/VINS-Fusion" ]]; then
  "${REPOSITORY_ROOT}/scripts/build_task2_workspace.sh"
fi

ln -sfn "${EGO_ROOT}" "${WORKSPACE}/src/ego-planner"
source "${THIRD_PARTY_ROOT}/AirSim/ros/devel/setup.bash"
cd "${WORKSPACE}"
catkin_make -DCMAKE_BUILD_TYPE=Release -DORB_SLAM3_ROOT="${ORB_SLAM3_ROOT}"

echo "EGO-Planner 与 AirSim 桥接工作空间构建完成。"
