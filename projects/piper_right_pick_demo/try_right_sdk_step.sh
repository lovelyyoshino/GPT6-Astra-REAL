#!/usr/bin/env bash
# Enable, plan from the actual held pose, move once, and capture front/right RGBD.
set -euo pipefail
demo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ $# -ne 0 ]]; then
  echo 'Usage: bash try_right_sdk_step.sh' >&2
  echo '旧 --reviewed-joint-reentry 已停用。此入口改为使能后按实时姿态计算目标，无需该参数。' >&2
  exit 2
fi
mapfile -t step_config < <(python3 - "$demo_root/configs/site.local.json" <<'PY'
import json, sys
with open(sys.argv[1], encoding='utf-8') as stream:
    config = json.load(stream)['host_capture']
for key in ('robot_python', 'camera_python'):
    value = config[key]
    if not isinstance(value, str) or '\n' in value or not value:
        raise ValueError('Invalid Python executable configuration: ' + key)
    print(value)
PY
)
if [[ ${#step_config[@]} -ne 2 ]]; then
  echo 'Missing host_capture Python configuration.' >&2
  exit 2
fi
mkdir -p "$demo_root/runs"
exec 9>"$demo_root/runs/direct_sdk_step.lock"
if ! flock -n 9; then
  echo '另一个 SDK 单步程序正在运行；本次退出。' >&2
  exit 2
fi
step_dir="$demo_root/runs/sdk_live_$(date -u +%Y%m%dT%H%M%S)_$$"
mkdir "$step_dir"
exec > >(tee -a "$step_dir/terminal.log") 2>&1
echo "本次记录：$step_dir"
link_action=$(python3 - <<'PY'
import json, pathlib, subprocess
device = pathlib.Path('/sys/class/net/can2/device')
if not device.exists() or device.resolve().name != '1-6.3:1.0':
    raise SystemExit('右臂 USB 绑定不是 can2 / 1-6.3:1.0；停止。')
link = json.loads(subprocess.check_output(
    ['ip', '-j', '-details', 'link', 'show', 'dev', 'can2'], text=True))[0]
kind = link.get('linkinfo', {}).get('info_kind')
info = link.get('linkinfo', {}).get('info_data', {})
if kind != 'can':
    raise SystemExit('can2 不是 CAN 接口；停止。')
if 'UP' in link.get('flags', []):
    if info.get('bittiming', {}).get('bitrate') != 1000000:
        raise SystemExit('can2 已运行但比特率不是 1 Mbps；停止，不重配运行中的总线。')
    print('ready')
else:
    blocked = []
    names = {'roslaunch', 'rosrun', 'piper_ctrl_single_node.py',
             'piper_ctrl_dual_node.py', 'piper_start_ms_node.py',
             'piper_control.py', 'start_2_piper.sh', 'start_3_piper.sh'}
    for proc in pathlib.Path('/proc').iterdir():
        if not proc.name.isdigit():
            continue
        try:
            args = (proc / 'cmdline').read_bytes().split(b'\0')
        except (FileNotFoundError, ProcessLookupError, PermissionError):
            continue
        basenames = {pathlib.Path(arg.decode(errors='replace')).name for arg in args if arg}
        matches = names.intersection(basenames)
        matches.update(name for name in basenames if name.startswith('piper_ctrl_'))
        if matches:
            blocked.append((proc.name, sorted(matches)))
    if blocked:
        raise SystemExit('已有控制程序，不能在其运行时开启总线：' + repr(blocked))
    print('activate')
PY
)
if [[ "$link_action" == activate ]]; then
  echo 'can2 当前 DOWN；恢复右臂 CAN 通信。sudo 若询问密码，请在本终端输入。'
  sudo ip link set dev can2 type can bitrate 1000000
  sudo ip link set dev can2 up
fi
echo '右臂 SDK：读取当前状态 → 使能并确认 → 按稳定实测姿态计算 15 mm 抬升 → 一次低速移动。'
step_status=0
"${step_config[0]}" "$demo_root/scripts/direct_sdk_live_step.py" --output-dir "$step_dir" || step_status=$?
if [[ "$step_status" -eq 130 ]]; then
  echo '用户已中断；不会继续拍摄。程序退出不等于已撤销机械臂目标。' >&2
  exit "$step_status"
fi
echo '保存执行后的前视与右腕两路画面、深度；相机采集不会再次移动机械臂。'
camera_status=0
"${step_config[1]}" "$demo_root/scripts/capture_scene.py" \
  --config "$demo_root/configs/site.local.json" --output-root "$step_dir/after" \
  --cameras front right_hand || camera_status=$?
echo "SDK 返回码：$step_status；相机返回码：$camera_status；结果目录：$step_dir"
if [[ "$step_status" -ne 0 ]]; then
  exit "$step_status"
fi
exit "$camera_status"
