#!/usr/bin/env bash
# One finite, locally started SDK pose batch; no daemon or remote command listener.
set -euo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -ne 0 ]]; then
  echo 'Usage: bash ~/piper_right_pick_demo/try_right_pose_batch.sh' >&2
  exit 2
fi
exec 9>"$demo_root/runs/direct_sdk_step.lock"
if ! flock -n 9; then
  echo '已有右臂 SDK 程序在运行，本次退出。' >&2
  exit 2
fi
attempt_dir="$demo_root/runs/pose_batch_$(date -u +%Y%m%dT%H%M%S)_$$"
mkdir "$attempt_dir"
exec > >(tee -a "$attempt_dir/terminal.log") 2>&1
exec /home/agilex/miniconda3/envs/aloha/bin/python3 \
  "$demo_root/scripts/run_sdk_pose_batch.py" \
  --plan "$demo_root/runs/pose_batch_20261003T212603_134296/continuation_pose_plan.json" \
  --output-dir "$attempt_dir"
