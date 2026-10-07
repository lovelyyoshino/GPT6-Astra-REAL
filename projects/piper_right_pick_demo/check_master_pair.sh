#!/usr/bin/env bash
# Human-run: activate only master CAN if needed, then receive both buses.
set -euo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -gt 1 || ( $# -eq 1 && "$1" != --observe-master-motion ) ]]; then
  echo 'Usage: bash check_master_pair.sh [--observe-master-motion]' >&2
  exit 2
fi
mapfile -t pair_config < <(python3 - "$demo_root/configs/site.local.json" <<'PY'
import json,sys
c=json.load(open(sys.argv[1]))['host_capture']
for k in ['robot_python','master_can_interface','master_usb_port','right_can_interface','right_usb_port','can_activate_script']:
 print(c[k])
PY
)
if [[ ${#pair_config[@]} -ne 6 ]]; then
  echo 'Missing reviewed CAN bindings.' >&2
  exit 2
fi
robot_python="${pair_config[0]}"
master_can="${pair_config[1]}"
master_usb="${pair_config[2]}"
right_can="${pair_config[3]}"
right_usb="${pair_config[4]}"
can_activate="${pair_config[5]}"
if [[ "$master_can" != can1 || "$right_can" != can2 ]]; then
  echo 'CAN names differ from reviewed master/right mapping.' >&2
  exit 2
fi
for pair in "$master_can $master_usb" "$right_can $right_usb"; do
  read -r pair_can pair_usb <<< "$pair"
  actual_device=$(readlink -f "/sys/class/net/$pair_can/device")
  if [[ "${actual_device##*/}" != "$pair_usb" ]]; then
    echo "USB binding mismatch: $pair_can" >&2
    exit 2
  fi
done
if pgrep -af '(^|/)[^ /]*piper[^ /]*\.py([[:space:]]|$)'; then
  echo '检测到 Piper 节点，保留其状态。本入口用于当前无桥接的诊断场景。' >&2
  exit 2
fi
master_state=$(cat "/sys/class/net/$master_can/operstate")
if [[ "$master_state" == down ]]; then
  echo '只开启主臂 can1 通信；不会改动右臂 can2，也不会启动主从跟随。'
  bash "$can_activate" "$master_can" 1000000 "$master_usb"
fi
# Check the resulting link and bitrate; never reconfigure can2 here.
for interface in "$master_can" "$right_can"; do
  ip -details link show dev "$interface" | python3 -c '
import re,sys
text=sys.stdin.read()
if not re.search(r"<[^>]*\bUP\b[^>]*>",text) or not re.search(r"\bbitrate\s+1000000(?:\s|$)",text):
 raise SystemExit("CAN link is not UP at 1 Mbit/s; no further changes made")
'
done
if [[ $# -eq 0 ]]; then
  echo '保持两臂姿态不变；同时被动采集主臂目标和右臂反馈，约 3 秒。'
fi
exec "$robot_python" "$demo_root/scripts/check_master_pair.py" "$@"
