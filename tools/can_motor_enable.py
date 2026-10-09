#!/usr/bin/env python3
"""Manual motor startup via existing ToolService contracts; no new CAN encoder.

--plan / --check / --check-startup never connect devices. --check validates
bindings only; --check-startup also admits a potential network/startup write.
The execution branch reads fresh state before admitting any motor command.
"""

import argparse
import json
from pathlib import Path
import sys
import uuid


ROOT = Path(__file__).resolve().parents[1]
PROJECT = ROOT / "projects/piperx_cloth_demo"
SIDES = ("left", "right")
LABELS = {"left": "左臂", "right": "右臂"}


def plan(selected, project=PROJECT):
    profile = json.loads((project / "configs/robot.json").read_text())
    configs = profile["arms"]
    if set(configs) != set(SIDES):
        raise RuntimeError("现有启动入口需要明确配置两臂，以监测未操作侧。")
    channels = {configs[side]["channel"]: side for side in SIDES}
    if len(channels) != 2:
        raise RuntimeError("两臂不能绑定同一 CAN 接口。")
    unknown = set(selected) - set(channels)
    if not selected or unknown:
        raise RuntimeError("接口没有机械臂绑定：{}；仅开通信请用 --can-only。".format(sorted(unknown)))
    return {"selected_arms": [side for side in SIDES if configs[side]["channel"] in selected],
            "required_interfaces": [configs[side]["channel"] for side in SIDES],
            "bindings": configs}


def check_bindings(selection):
    for side, cfg in selection["bindings"].items():
        device = Path("/sys/class/net") / cfg["channel"]
        if (device / "type").read_text().strip() != "280":
            raise RuntimeError("非 CAN 接口：" + cfg["channel"])
        actual = (device / "device").resolve(strict=True).name
        if actual != cfg["usb_interface"]:
            raise RuntimeError("{} USB 绑定不一致：配置 {}，实际 {}。".format(
                LABELS[side], cfg["usb_interface"], actual))


def check_ownership(project=PROJECT):
    from robot_tools.pair_ledger import platform_state
    from robot_tools.reboot_startup import HostBootRequired, check_processes, inspect
    from robot_tools.startup_reset import inspect as inspect_reset
    reset_state = inspect_reset(project)
    if reset_state is not None:
        check_processes()
        if reset_state["status"] == "complete":
            # The reset latch protects its first startup, not every later
            # physical power cycle in this Linux boot. Keep that receipt and
            # use the existing independently claimed, confirmed cycle route.
            from robot_tools.arm_power_cycle import inspect as inspect_arm_cycle
            return inspect_arm_cycle(project)
        return reset_state
    # Source projects can still own hardware. Read their ledgers too; never
    # clear owners/faults or switch to a different project to evade a refusal.
    roots = {project.resolve(), Path("/home/agilex/piperx_cloth_demo").resolve()}
    blocked = False
    for root in sorted(roots):
        database = root / "runs/pair_sessions.sqlite"
        state = platform_state(database)
        if state and (state["owner"] or state["fault"] or state["pending_events"]):
            blocked = True
    if blocked:
        from robot_tools.arm_power_cycle import has_startup_history, inspect as inspect_arm_cycle
        if has_startup_history(project):
            return inspect_arm_cycle(project)
        try:
            return inspect(project)
        except HostBootRequired:
            return inspect_arm_cycle(project)
    check_processes()
    return {"route": "ordinary_startup"}


def confirm_reboot_scene():
    from robot_tools.reboot_startup import CONFIRMATION
    print("检测到电脑重启前的历史任务记录，将保留历史并使用独立的电机启动流程。", flush=True)
    if not sys.stdin.isatty():
        raise RuntimeError("重启接续需要现场确认，请在终端交互运行 ./can_enable.sh。")
    answer = input(CONFIRMATION + "。确认请输入 yes，其他输入退出：").strip().lower()
    if answer != "yes":
        raise RuntimeError("未确认机械臂断电重启和当前现场条件，没有发送使能命令。")
    return CONFIRMATION


def confirm_reset_scene():
    from robot_tools.reboot_startup import CONFIRMATION
    print("旧任务软件状态已归档重置；物体是否脱离、硬件是否正常仍按当前现场和反馈核对。", flush=True)
    if not sys.stdin.isatty():
        raise RuntimeError("软件重置已完成；电机使能需要现场核对，请交互运行 ./can_enable.sh。")
    if input(CONFIRMATION + "。确认请输入 yes，其他输入退出：").strip().lower() != "yes":
        raise RuntimeError("软件状态已重置，未确认当前启动条件，没有发送使能命令。")
    return CONFIRMATION


def confirm_arm_cycle_scene(arm):
    from robot_tools.arm_power_cycle import confirmation
    statement = confirmation(arm)
    print("支持只给机械臂断电：保留旧任务和故障，单独记录本次启动，无需重启电脑。", flush=True)
    if not sys.stdin.isatty():
        raise RuntimeError("机械臂断电接续需要现场确认，请在终端交互运行 ./can_enable.sh。")
    if input(statement + "。确认请输入 yes，其他输入退出：").strip().lower() != "yes":
        raise RuntimeError("未确认本次机械臂断电和现场条件，没有发送使能命令。")
    return "arm_cycle_" + uuid.uuid4().hex, statement


def startup_route(states, selected):
    """Use current telemetry only to route; startup rechecks on its own socket."""
    pending = []
    for side in selected:
        state = states[side]
        flags = [state["drivers"][str(i)]["foc_status"].get("driver_enable_status")
                 for i in range(1, 7)]
        jaw = state["gripper"]["foc_status"].get("driver_enable_status")
        mode = state["arm_status"].get("ctrl_mode")
        if (any(type(flag) is not bool for flag in flags + [jaw])
                or not isinstance(mode, int) or isinstance(mode, bool)):
            raise RuntimeError(LABELS[side] + "使能位或控制模式未知，停止操作。")
        print("{}：控制模式={}，关节使能={}/6，夹爪使能={}。".format(
            LABELS[side], mode, sum(flags), jaw), flush=True)
        if mode == 1 and all(flags):
            print(LABELS[side] + "关节电机已使能，跳过发送。", flush=True)
        elif mode == 0 and not any(flags + [jaw]):
            pending.append(side)
        else:
            raise RuntimeError(LABELS[side] + "不是完整待机失能状态或已就绪状态；"
                               "不重放部分启动，不自动切示教模式或复位。")
    if len(pending) == 2:
        return "robot_startup_arms", {}, pending
    if len(pending) == 1:
        return "robot_startup_arm", {"arm": pending[0]}, pending
    return None, {}, []


def startup_failure_summary(result, selected):
    """Separate observed enable bits from the outcome of the startup guard.

    These are historical receipt observations, not a new live state check or
    permission to retry. The full result remains in the original run journal.
    """
    lines = ["启动检查未通过（{}），未自动重试。".format(result.get("status", "unknown"))]
    for side in selected:
        state = result.get("after", {}).get(side, {})
        flags = [state.get("drivers", {}).get(str(i), {}).get("foc_status", {}).get("driver_enable_status")
                 for i in range(1, 7)]
        if state.get("status") != "complete" or any(type(flag) is not bool for flag in flags):
            lines.append(LABELS[side] + "：末次回执不完整，关节使能状态未确认。")
            continue
        mode = state.get("arm_status", {}).get("ctrl_mode", "未知")
        lines.append("{}：末次回执关节使能={}/6，控制模式={}。".format(LABELS[side], sum(flags), mode))
    for error in result.get("errors", []):
        detail = error.get("detail", error.get("type", "未知错误")) if isinstance(error, dict) else error
        lines.append("检查原因：" + str(detail))
    if result.get("drift"):
        lines.append("偏移以本次使能前的位置为基准，不要求回原点。")
    lines.append("使能位不代表位置保持已验证；启动失败也不代表电机已失能或机械臂已停止。")
    if result.get("record_path"):
        lines.append("完整回执：" + str(result["record_path"]))
    return "\n".join(lines)


def execute(service, selection, ownership_check=None, scene_confirmation=confirm_reboot_scene,
            arm_cycle_confirmation=confirm_arm_cycle_scene, reset_confirmation=confirm_reset_scene):
    from robot_tools.arms import control_health
    if ownership_check is None:
        ownership_check = check_ownership
    # robot_read_state is a TX-blocked reader, not a new control host. Failed
    # send claims must block another send, not observation of enabled motors.
    result = service.call("robot_read_state", {})
    if not result.get("ok") or result.get("state", {}).get("status") != "complete":
        raise RuntimeError("无法取得完整双臂反馈：" + json.dumps(result, ensure_ascii=False))
    states = result["state"]["arms"]
    for side in SIDES:
        state = states[side]
        # Validate the captured sample at its actual timestamp. A subsequent
        # startup independently acquires new live samples before every TX.
        health = control_health(state, now_s=state["timestamp"],
                                allowed_control_modes=(0, 1, 2), require_enabled=False)
        if state.get("status") != "complete" or not health["healthy"]:
            raise RuntimeError(LABELS[side] + "反馈不健康：" + json.dumps(health, ensure_ascii=False))
    tool, arguments, pending = startup_route(states, selection["selected_arms"])
    if tool is None:
        print("所选关节电机均已使能；本次仅查看反馈，未发送使能命令。"
              "原有故障记录保留，位置保持尚未验证。", flush=True)
        return
    route = ownership_check()
    if isinstance(route, dict) and route.get("route") == "state_reset_startup":
        if route.get("status") != "awaiting_startup":
            raise RuntimeError("本次软件重置已完成过启动；不会重复发送使能命令。")
        tool = "robot_startup_after_state_reset"
        arguments = {"arm": "both" if len(pending) == 2 else pending[0],
                     "power_cycle_and_clearance_statement": reset_confirmation()}
    elif isinstance(route, dict) and route.get("route") == "reboot_startup":
        tool = "robot_startup_after_host_reboot"
        arguments = {"arm": "both" if len(pending) == 2 else pending[0],
                     "power_cycle_and_clearance_statement": scene_confirmation()}
    elif isinstance(route, dict) and route.get("route") == "arm_power_cycle_startup":
        arm = "both" if len(pending) == 2 else pending[0]
        cycle_id, statement = arm_cycle_confirmation(arm)
        tool = "robot_startup_after_arm_power_cycle"
        arguments = {"arm": arm, "power_cycle_id": cycle_id,
                     "power_cycle_and_clearance_statement": statement}
    print("通过现有启动入口使能 {}；原入口将检查静止、模式和逐关节反馈。".format(
        "、".join(LABELS[side] for side in pending)), flush=True)
    print("以使能前的当前位置检查意外偏移，无需回原点；本脚本不发送回零目标。", flush=True)
    result = service.call(tool, arguments)
    record = result.get("record_path")
    if record:
        print("启动回执：" + record, flush=True)
    if result.get("ok") is not True:
        raise RuntimeError(startup_failure_summary(result, pending))
    for side in pending:
        state = result.get("after", {}).get(side, {})
        flags = [state.get("drivers", {}).get(str(i), {}).get("foc_status", {}).get("driver_enable_status")
                 for i in range(1, 7)]
        if (state.get("status") != "complete" or state.get("arm_status", {}).get("ctrl_mode") != 1
                or not all(flag is True for flag in flags)):
            raise RuntimeError(LABELS[side] + "启动回执缺少完整 CAN 模式/六关节使能证据，未重试。")
        print(LABELS[side] + "：六个关节电机使能已确认（6/6），CAN 控制模式已确认。", flush=True)
    print("本次未发送关节位置或归零目标；使能回执不等于位置保持验证。", flush=True)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--plan", action="store_true")
    mode.add_argument("--check", action="store_true")
    mode.add_argument("--check-startup", action="store_true")
    mode.add_argument("--reset-state", action="store_true", help="归档并重置旧任务软件状态；不连接设备")
    parser.add_argument("interfaces", nargs="+")
    args = parser.parse_args(argv)
    selection = plan(args.interfaces)
    if args.plan:
        print(json.dumps(selection, ensure_ascii=False))
        return 0
    sys.path.insert(0, str(PROJECT))
    check_bindings(selection)
    if args.reset_state:
        from robot_tools.startup_reset import reset
        print(json.dumps(reset(PROJECT), ensure_ascii=False))
        return 0
    if args.check or args.check_startup:
        selection["startup_route"] = (check_ownership()["route"] if args.check_startup
                                      else "deferred_until_fresh_state")
        print(json.dumps(selection, ensure_ascii=False))
        return 0
    from robot_tools.service import ToolService
    # The original service journals requests, sends and raw feedback in runs/.
    execute(ToolService(PROJECT), selection)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print("电机启动流程未通过：" + str(exc), file=sys.stderr)
        raise SystemExit(1)
    except KeyboardInterrupt:
        print("使能操作被中断；未重试，不能据进程退出判断机械臂状态。", file=sys.stderr)
        raise SystemExit(130)
