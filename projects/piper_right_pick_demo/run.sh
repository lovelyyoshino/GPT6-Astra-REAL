#!/usr/bin/env bash
set -euo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$demo_root"
export PYTHONPATH="$demo_root/src${PYTHONPATH:+:$PYTHONPATH}"
exec "${DEMO_PYTHON:-python3}" -m right_pick.cli "$@"
