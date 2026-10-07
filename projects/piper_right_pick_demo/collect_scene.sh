#!/usr/bin/env bash
# Human-run single observation. Optional CAN link activation sends no arm commands.
set -euo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
activate_right=false
if [[ "${1:-}" == --activate-right-can ]]; then
  activate_right=true
elif [[ $# -gt 0 ]]; then
  echo 'Usage: bash collect_scene.sh [--activate-right-can]' >&2
  exit 2
fi
mapfile -t scene_config < <(python3 - "$demo_root/configs/site.local.json" <<'PY'
import json,sys
c=json.load(open(sys.argv[1]))['host_capture']
for k in ['camera_python','robot_python','right_can_interface','right_usb_port','can_activate_script']:
 print(c[k])
PY
)
if [[ ${#scene_config[@]} -ne 5 ]]; then
  echo 'Missing host_capture site configuration.' >&2
  exit 2
fi
camera_python="${scene_config[0]}"
robot_python="${scene_config[1]}"
right_can="${scene_config[2]}"
right_usb="${scene_config[3]}"
can_activate="${scene_config[4]}"
if [[ "$right_can" != can2 ]]; then
  echo 'Site mapping mismatch: expected the reviewed right-arm can2.' >&2
  exit 2
fi
if [[ "$activate_right" == true ]]; then
  if pgrep -af 'piper_(ctrl_single_node|start_ms_node|control)\.py' ; then
    echo '检测到现有 Piper 控制节点；先核对其运行状态，本脚本不重配 CAN。' >&2
    exit 2
  fi
  can_device=$(readlink -f "/sys/class/net/$right_can/device")
  if [[ "${can_device##*/}" != "$right_usb" ]]; then
    echo 'Right CAN USB mapping differs from the reviewed site binding; stopping.' >&2
    exit 2
  fi
  echo '只配置右臂 CAN 通信接口；sudo 密码请在此终端输入。不会使能、回零或发送机械臂指令。'
  bash "$can_activate" "$right_can" 1000000 "$right_usb"
fi
mkdir -p "$demo_root/runs"
snapshot_path="$demo_root/runs/passive_right_$(date -u +%Y%m%dT%H%M%S)_$$.json"
echo '被动读取右臂反馈（不会发送 CAN）；接口未启动时保留错误并继续相机采集。'
if ! "$robot_python" "$demo_root/scripts/passive_can_snapshot.py" --channel "$right_can" --pose-trace > "$snapshot_path"; then
  echo "右臂反馈未就绪；详见 $snapshot_path"
fi
echo "右臂反馈报告：$snapshot_path"
echo '采集前视与右腕两路 RGB/深度。相机被占用时停止并报告，不结束其他进程。'
exec "$camera_python" "$demo_root/scripts/capture_scene.py" --config "$demo_root/configs/site.local.json"
