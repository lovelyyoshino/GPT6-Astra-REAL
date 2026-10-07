#!/usr/bin/env bash
# Local human-operated evidence collection only; no controller or CAN setup.
set -euo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
camera_python=$(python3 - "$demo_root/configs/site.local.json" <<'PY'
import json, sys
print(json.load(open(sys.argv[1]))['host_capture']['camera_python'])
PY
)
exec "$camera_python" "$demo_root/scripts/capture_alignment_pose.py" --config "$demo_root/configs/site.local.json" "$@"
