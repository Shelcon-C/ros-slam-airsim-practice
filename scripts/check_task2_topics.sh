#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "noetic" ]]; then
  echo "请先 source /opt/ros/noetic/setup.bash 和 catkin_ws/devel/setup.bash。" >&2
  exit 1
fi
if ! rosnode list >/dev/null 2>&1; then
  echo "无法连接 ROS Master；请先启动 task2_airsim_vins.launch。" >&2
  exit 1
fi

required_topics=(
  /vins_fusion/cam0/image_raw
  /vins_fusion/cam1/image_raw
  /vins_fusion/imu
  /vins_estimator/odometry
  /slam_practice/vins/odometry
  /slam_practice/vins/path
)

missing=0
for topic in "${required_topics[@]}"; do
  if rostopic info "${topic}" >/dev/null 2>&1; then
    echo "[存在] ${topic}"
  else
    echo "[缺失] ${topic}" >&2
    missing=1
  fi
done

if [[ "${missing}" -ne 0 ]]; then
  echo "存在缺失 Topic，先检查 AirSim Wrapper、桥接节点和 VINS-Fusion 终端日志。" >&2
  exit 2
fi

echo "采样频率（每项最多等待 8 秒）："
for topic in /vins_fusion/cam0/image_raw /vins_fusion/cam1/image_raw /vins_fusion/imu /slam_practice/vins/odometry; do
  echo "--- ${topic}"
  timeout 8 rostopic hz "${topic}" -w 20 || true
done

echo "VINS 输出样本："
timeout 5 rostopic echo -n 1 /slam_practice/vins/odometry || {
  echo "未在 5 秒内收到 VINS 位姿。" >&2
  exit 3
}
