#!/usr/bin/env bash
set -euo pipefail

REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY_ROOT="${THIRD_PARTY_ROOT:-${REPOSITORY_ROOT}/third_party}"
WORKSPACE="${REPOSITORY_ROOT}/catkin_ws"

if [[ "${ROS_DISTRO:-}" != "noetic" ]]; then
  echo "请先执行：source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi
if [[ ! -d "${THIRD_PARTY_ROOT}/ORB_SLAM3" ]]; then
  echo "未找到 ORB-SLAM3，请先运行 scripts/fetch_third_party.sh" >&2
  exit 1
fi

if [[ ! -f "${THIRD_PARTY_ROOT}/Pangolin/build/src/libpangolin.so" ]]; then
  cmake -S "${THIRD_PARTY_ROOT}/Pangolin" -B "${THIRD_PARTY_ROOT}/Pangolin/build" \
    -DCMAKE_BUILD_TYPE=Release -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=OFF
  cmake --build "${THIRD_PARTY_ROOT}/Pangolin/build" --parallel "$(nproc)"
  sudo cmake --install "${THIRD_PARTY_ROOT}/Pangolin/build"
fi

if [[ ! -f "${THIRD_PARTY_ROOT}/ORB_SLAM3/lib/libORB_SLAM3.so" ]]; then
  (cd "${THIRD_PARTY_ROOT}/ORB_SLAM3" && ./build.sh)
fi

cd "${WORKSPACE}"
catkin_make -DCMAKE_BUILD_TYPE=Release -DORB_SLAM3_ROOT="${THIRD_PARTY_ROOT}/ORB_SLAM3"
echo "构建完成。新终端请执行：source ${WORKSPACE}/devel/setup.bash"
