#!/usr/bin/env bash
set -euo pipefail

REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
THIRD_PARTY_ROOT="${THIRD_PARTY_ROOT:-${REPOSITORY_ROOT}/third_party}"
mkdir -p "${THIRD_PARTY_ROOT}"

clone_if_missing() {
  local url="$1"
  local destination="$2"
  shift 2
  if [[ -d "${destination}/.git" ]]; then
    echo "已存在，跳过：${destination}"
    return
  fi
  git clone --depth 1 "$@" "${url}" "${destination}"
}

clone_if_missing https://github.com/stevenlovegrove/Pangolin.git "${THIRD_PARTY_ROOT}/Pangolin" --branch v0.6
clone_if_missing https://github.com/UZ-SLAMLab/ORB_SLAM3.git "${THIRD_PARTY_ROOT}/ORB_SLAM3"
clone_if_missing https://github.com/HKUST-Aerial-Robotics/VINS-Fusion.git "${THIRD_PARTY_ROOT}/VINS-Fusion"
clone_if_missing https://github.com/microsoft/AirSim.git "${THIRD_PARTY_ROOT}/AirSim"
clone_if_missing https://github.com/ZJU-FAST-Lab/ego-planner.git "${THIRD_PARTY_ROOT}/ego-planner"

echo "第三方源码已放置在 ${THIRD_PARTY_ROOT}。"
