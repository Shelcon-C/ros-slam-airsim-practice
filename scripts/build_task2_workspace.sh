#!/usr/bin/env bash
set -euo pipefail

REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY_ROOT="${THIRD_PARTY_ROOT:-${REPOSITORY_ROOT}/third_party}"
WORKSPACE="${REPOSITORY_ROOT}/catkin_ws"
AIRSIM_ROOT="${THIRD_PARTY_ROOT}/AirSim"
VINS_ROOT="${THIRD_PARTY_ROOT}/VINS-Fusion"
ORB_SLAM3_ROOT="${THIRD_PARTY_ROOT}/ORB_SLAM3"

if [[ "${ROS_DISTRO:-}" != "noetic" ]]; then
  echo "请先执行：source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi
for dependency in "${AIRSIM_ROOT}" "${VINS_ROOT}" "${ORB_SLAM3_ROOT}"; do
  if [[ ! -d "${dependency}" ]]; then
    echo "缺少 ${dependency}，请先运行 scripts/fetch_third_party.sh。" >&2
    exit 1
  fi
done

# AirSim 主库和 ros wrapper 使用上游脚本/工作空间独立构建。
if [[ ! -f "${AIRSIM_ROOT}/AirLib/lib/x64/Release/libAirLib.a" ]]; then
  (cd "${AIRSIM_ROOT}" && ./setup.sh && ./build.sh)
fi
(cd "${AIRSIM_ROOT}/ros" && catkin build)

# VINS-Fusion 以源码包形式加入本项目 catkin 工作空间；软链接内容由 .gitignore 排除。
ln -sfn "${VINS_ROOT}" "${WORKSPACE}/src/VINS-Fusion"

if [[ ! -f "${ORB_SLAM3_ROOT}/lib/libORB_SLAM3.so" ]]; then
  "${REPOSITORY_ROOT}/scripts/build_workspace.sh"
fi

source "${AIRSIM_ROOT}/ros/devel/setup.bash"
cd "${WORKSPACE}"
catkin_make -DCMAKE_BUILD_TYPE=Release -DORB_SLAM3_ROOT="${ORB_SLAM3_ROOT}"

echo "任务二工作空间构建完成。"
echo "执行：source ${AIRSIM_ROOT}/ros/devel/setup.bash"
echo "再执行：source ${WORKSPACE}/devel/setup.bash"
