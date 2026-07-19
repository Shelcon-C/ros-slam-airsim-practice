#!/usr/bin/env bash
set -euo pipefail

REPOSITORY_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
EVO_VENV="${REPOSITORY_ROOT}/.venv-evo"

if [[ "${ROS_DISTRO:-}" != "noetic" ]]; then
  echo "请先安装并 source ROS Noetic：source /opt/ros/noetic/setup.bash" >&2
  exit 1
fi

sudo apt-get update
sudo apt-get install -y \
  build-essential cmake git curl unzip pkg-config \
  libarmadillo-dev libboost-all-dev libceres-dev libeigen3-dev libglew-dev libopencv-dev libssl-dev \
  python3-opencv python3-pip python3-rosdep python3-catkin-tools python3-venv \
  ros-noetic-camera-info-manager ros-noetic-cv-bridge ros-noetic-image-transport \
  ros-noetic-message-filters ros-noetic-rviz ros-noetic-tf2-ros

if [[ ! -x "${EVO_VENV}/bin/python" ]]; then
  python3 -m venv --system-site-packages "${EVO_VENV}"
fi
"${EVO_VENV}/bin/python" -m pip install "evo==1.30.6"

echo "ROS Noetic 实践依赖安装完成。"
echo "评估轨迹前执行：source ${EVO_VENV}/bin/activate"
