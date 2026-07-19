#!/usr/bin/env bash
set -euo pipefail

REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
DATASET_ROOT="${DATASET_ROOT:-${REPOSITORY_ROOT}/datasets/euroc}"
SEQUENCE="${1:-MH_01_easy}"
case "${SEQUENCE}" in
  MH_01_easy) FOLDER="machine_hall" ;;
  MH_02_easy) FOLDER="machine_hall" ;;
  MH_03_medium) FOLDER="machine_hall" ;;
  V1_01_easy) FOLDER="vicon_room1" ;;
  *) echo "暂不支持自动下载序列：${SEQUENCE}" >&2; exit 1 ;;
esac

mkdir -p "${DATASET_ROOT}"
ARCHIVE="${DATASET_ROOT}/${SEQUENCE}.zip"
URL="https://robotics.ethz.ch/~asl-datasets/ijrr_euroc_mav_dataset/${FOLDER}/${SEQUENCE}/${SEQUENCE}.zip"
curl --fail --location --retry 3 "${URL}" --output "${ARCHIVE}"
unzip -q -o "${ARCHIVE}" -d "${DATASET_ROOT}/${SEQUENCE}"
echo "EuRoC ${SEQUENCE} 已解压到 ${DATASET_ROOT}/${SEQUENCE}。"
