#!/usr/bin/env bash
# Run in a visible terminal. Subscribe only; never launch/enable a robot driver.
set -euo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ -f /opt/ros/noetic/setup.bash ]]; then
  set +u
  source /opt/ros/noetic/setup.bash
  set -u
fi
site_setup="${PIPER_ROS_SETUP:-}"
if [[ -z "$site_setup" ]]; then
  site_setup=$(python3 -c 'import json,sys; print(json.load(open(sys.argv[1])).get("robot",{}).get("ros_setup") or "")' "$demo_root/configs/site.local.json")
fi
if [[ -n "$site_setup" ]]; then
  set +u
  source "$site_setup"
  set -u
fi
bash "$demo_root/run.sh" observe
