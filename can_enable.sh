#!/usr/bin/env bash
# 归档重置旧软件状态后开启 CAN/关节电机；--reset-only 不操作硬件。
# --list 仅查看，--can-only 仅开通信，--keep-state 保留旧状态。
set -euo pipefail
SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
exec python3 "$SCRIPT_DIR/tools/can_enable.py" "$@"
