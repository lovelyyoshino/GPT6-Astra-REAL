#!/usr/bin/env bash
# Run in the previously working terminal; no sourcing/startup/enable/ROS writes.
set -euo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec "${DEMO_PYTHON:-python3}" "$demo_root/scripts/host_probe.py"
