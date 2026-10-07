#!/usr/bin/env bash
set -euo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$demo_root"
export PYTHONPATH="$demo_root/src${PYTHONPATH:+:$PYTHONPATH}"
"${DEMO_PYTHON:-python3}" -m unittest discover -s tests -v
bash run.sh replay
