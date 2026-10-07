#!/usr/bin/env bash
# Finish the reviewed pre-close trajectory that stopped at waypoint 60.
set -euo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -ne 0 ]]; then
  echo 'Usage: bash ~/piper_right_pick_demo/resume_right_pick.sh' >&2
  exit 2
fi
robot_python=$(python3 - "$demo_root/configs/site.local.json" <<'PY'
import json,sys
value=json.load(open(sys.argv[1]))['host_capture']['robot_python']
if not isinstance(value,str) or not value or '\n' in value:
    raise ValueError('Invalid robot Python configuration')
print(value)
PY
)
source_report="$demo_root/runs/pick_attempt_20261003T194811_102398/report.json"
test -f "$source_report"
exec 9>"$demo_root/runs/direct_sdk_step.lock"
if ! flock -n 9; then
  echo '已有右臂 SDK 程序在运行，本次退出。' >&2
  exit 2
fi
attempt_dir="$demo_root/runs/pick_resume_$(date -u +%Y%m%dT%H%M%S)_$$"
mkdir "$attempt_dir"
exec > >(tee -a "$attempt_dir/terminal.log") 2>&1
echo "本次剩余抓放记录：$attempt_dir"
exec "$robot_python" "$demo_root/scripts/run_right_pick.py" \
  --config "$demo_root/configs/site.local.json" --output-dir "$attempt_dir" \
  --resume-from "$source_report"
