#!/usr/bin/env bash
set -euo pipefail

if [[ "${ROS_DISTRO:-}" != "noetic" ]]; then
  echo "请先 source ROS、AirSim 与项目工作空间。" >&2
  exit 1
fi

required_topics=(
  /slam_practice/vins/odometry
  /ego_bridge/depth
  /ego_bridge/camera_pose
  /move_base_simple/goal
  /planning/pos_cmd
  /airsim_node/vel_cmd_world_frame
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

if ! rosservice info /ego_position_controller/set_enabled >/dev/null 2>&1; then
  echo "[缺失] /ego_position_controller/set_enabled" >&2
  missing=1
else
  echo "[存在] /ego_position_controller/set_enabled"
fi

if [[ "${missing}" -ne 0 ]]; then
  exit 2
fi

echo "深度与相机位姿频率："
timeout 8 rostopic hz /ego_bridge/depth -w 20 || true
timeout 8 rostopic hz /ego_bridge/camera_pose -w 20 || true
echo "控制器仍应保持禁用。验证规划后再手动执行："
echo 'rosservice call /ego_position_controller/set_enabled "data: true"'
echo "紧急停止命令："
echo 'rosservice call /ego_position_controller/set_enabled "data: false"'
