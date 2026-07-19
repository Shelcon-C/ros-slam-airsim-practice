#!/usr/bin/env bash
set -euo pipefail

if [[ "$(uname -r | tr '[:upper:]' '[:lower:]')" != *microsoft* ]]; then
  echo "警告：当前内核看起来不是 WSL2；仍继续解析主机地址。" >&2
fi

if [[ ! -r /etc/resolv.conf ]]; then
  echo "无法读取 /etc/resolv.conf，不能自动发现 Windows 主机地址。" >&2
  exit 1
fi

WSL_HOST_IP="$(awk '/^nameserver[[:space:]]+/ {print $2; exit}' /etc/resolv.conf)"
if [[ -z "${WSL_HOST_IP}" ]]; then
  echo "未在 /etc/resolv.conf 找到 nameserver。" >&2
  exit 1
fi

echo "检测到 Windows 主机地址：${WSL_HOST_IP}"
echo "请在当前终端执行："
printf 'export WSL_HOST_IP=%q\n' "${WSL_HOST_IP}"

if timeout 2 bash -c ">/dev/tcp/${WSL_HOST_IP}/41451" 2>/dev/null; then
  echo "AirSim RPC 端口 ${WSL_HOST_IP}:41451 可连接。"
else
  echo "AirSim RPC 端口 ${WSL_HOST_IP}:41451 暂不可连接。请先启动 Windows 端 AirSim，并检查防火墙。" >&2
  exit 2
fi
