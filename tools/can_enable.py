#!/usr/bin/env python3
"""Enable SocketCAN, then delegate motor startup to the existing guarded tools."""

import argparse
import json
import os
from pathlib import Path
import shlex
import shutil
import subprocess
import sys


class CanError(RuntimeError):
    pass


def motor_command(selected, *, check=False):
    helper = Path(__file__).resolve().with_name("can_motor_enable.py")
    python = Path("/home/agilex/miniconda3/envs/pi0_infer/bin/python3.10")
    if not python.is_file():
        raise CanError("未找到项目现有 pi0_infer Python 3.10，无法调用电机使能入口。")
    return [str(python), "-B", str(helper)] + (["--check"] if check else []) + list(selected)


def prepare_motors(selected, bitrate, dry_run):
    if bitrate != 1000000:
        raise CanError("机械臂启动使用 1000000 bit/s；其他 CAN 配置请加 --can-only。")
    command = motor_command(selected, check=True)
    if dry_run:
        # Plan resolves configuration only; --check also validates USB bindings.
        # Motor write admission is deferred until after the live read.
        command[command.index("--check")] = "--plan"
    return json.loads(run(command, capture=True))["required_interfaces"]


def run(command, capture=False):
    result = subprocess.run(command, text=True, stdout=subprocess.PIPE if capture else None,
                            stderr=subprocess.PIPE if capture else None)
    if result.returncode:
        detail = (result.stderr or "").strip()
        raise CanError("命令失败（{}）：{}{}".format(
            result.returncode, shlex.join(command), "\n" + detail if detail else ""))
    return result.stdout


def interfaces(ip):
    links = json.loads(run([ip, "-j", "-details", "link", "show"], capture=True))
    return {link["ifname"]: link for link in links
            if link.get("linkinfo", {}).get("info_kind") == "can"}


def status(link):
    data = link.get("linkinfo", {}).get("info_data", {})
    return ("UP" in link.get("flags", []),
            data.get("bittiming", {}).get("bitrate"), data.get("state", "未知"))


def show(links):
    if not links:
        print("未发现 CAN 接口。请检查 USB-CAN 连接，再运行 ./can_enable.sh --list。")
        return
    print("序号  接口          开关   比特率(bit/s)  CAN 状态        设备路径")
    for index, (name, link) in enumerate(links.items(), 1):
        up, bitrate, state = status(link)
        device = Path("/sys/class/net") / name / "device"
        bus = str(device.resolve()) if device.exists() else "未知"
        print("{:<5} {:<13} {:<6} {:<14} {:<15} {}".format(
            index, name, "UP" if up else "DOWN", bitrate or "未配置", state, bus))
    print("接口名不代表左右臂；请按当前接线和设备路径区分。")


def needs_enable(link, bitrate):
    up, current, state = status(link)
    name = link["ifname"]
    if state not in ("STOPPED", "ERROR-ACTIVE"):
        raise CanError("{} 的 CAN 状态为 {}，停止操作，请先检查总线故障。".format(name, state))
    if up:
        if state != "ERROR-ACTIVE" or current != bitrate:
            raise CanError("{} 已开启，但状态/比特率与请求不一致（{} / {}）。"
                           "为避免中断现有通信，本脚本不会关闭、重配或重启它。".format(
                               name, state, current))
        return False
    return True


def enable(ip, selected, snapshot, bitrate, dry_run):
    # Validate the complete selection before changing any interface.
    for name in selected:
        if name not in snapshot:
            raise CanError("未发现 CAN 接口：{}。用 --list 查看当前接口。".format(name))
    pending = [name for name in selected if needs_enable(snapshot[name], bitrate)]
    if dry_run:
        for name in selected:
            if name not in pending:
                print("{} 已开启，且比特率为 {}；无需操作。".format(name, bitrate))
                continue
            for suffix in (["type", "can", "bitrate", str(bitrate)], ["up"]):
                print("预览：" + shlex.join(["sudo", ip, "link", "set", "dev", name] + suffix))
        return

    prefix = []
    if pending and os.geteuid() != 0:
        sudo = shutil.which("sudo")
        if not sudo:
            raise CanError("未找到 sudo；请由管理员运行此脚本。")
        print("需要管理员权限；如有密码提示，请输入当前用户的 sudo 密码（输入不回显）。", flush=True)
        run([sudo, "-v"])
        prefix = [sudo, "-n"]

    for name in selected:
        # Password entry may take time: reread before each interface change.
        current = interfaces(ip).get(name)
        if current is None or current["ifindex"] != snapshot[name]["ifindex"]:
            raise CanError("{} 已断开或设备身份发生变化，请重新查看接口。".format(name))
        if not needs_enable(current, bitrate):
            print("{} 已开启，且比特率为 {}；无需操作。".format(name, bitrate))
            continue
        if name not in pending:
            raise CanError("{} 从开启变为关闭，请重新查看当前接口后再操作。".format(name))
        print("启用 {}，比特率 {} bit/s……".format(name, bitrate), flush=True)
        run(prefix + [ip, "link", "set", "dev", name, "type", "can", "bitrate", str(bitrate)])
        # A failed configuration/up is reported as-is, without retry or rollback.
        run(prefix + [ip, "link", "set", "dev", name, "up"])
        after = interfaces(ip).get(name)
        if (after is None or after["ifindex"] != current["ifindex"]
                or status(after) != (True, bitrate, "ERROR-ACTIVE")):
            raise CanError("{} 执行后状态未确认正常；请运行 --list 查看，未自动重试。".format(name))
        print("{} 已确认：UP，{} bit/s，ERROR-ACTIVE。".format(name, bitrate))


def main(argv=None):
    parser = argparse.ArgumentParser(
        prog="./can_enable.sh",
        description="开启 CAN 通信并使能所选机械臂的关节电机，默认 1000000 bit/s。",
        epilog="示例：./can_enable.sh --list；./can_enable.sh can0 can1；./can_enable.sh --all")
    parser.add_argument("interfaces", nargs="*", metavar="接口", help="当前接口名称，例如 can0 can1")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--list", action="store_true", help="仅显示接口，无需 sudo")
    mode.add_argument("--all", action="store_true", help="启用当前所有 CAN 接口")
    parser.add_argument("--bitrate", type=int, default=1000000, help="比特率，默认 1000000 bit/s")
    parser.add_argument("--dry-run", action="store_true", help="仅预览命令，不修改接口或请求 sudo")
    parser.add_argument("--can-only", action="store_true", help="只开启 CAN 通信，不使能关节电机")
    parser.add_argument("--keep-state", action="store_true", help="保留旧任务软件状态，使用原接续检查")
    parser.add_argument("--reset-only", action="store_true", help="只归档重置旧任务软件状态，不开启 CAN 或使能电机")
    args = parser.parse_args(argv)
    if (args.list or args.all) and args.interfaces:
        parser.error("--list / --all 不能与接口名称同时使用")
    if not 1 <= args.bitrate <= 1000000:
        parser.error("经典 CAN 比特率必须在 1..1000000 bit/s 内")
    if args.reset_only and (args.can_only or args.keep_state or args.list):
        parser.error("--reset-only 不能与 --can-only、--keep-state 或 --list 同用")
    ip = shutil.which("ip")
    if not ip:
        raise CanError("未找到 ip 命令，请安装系统 iproute2 软件包。")
    links = interfaces(ip)
    show(links)
    if args.list:
        return 0
    if not links:
        return 1
    selected = list(links) if args.all else args.interfaces
    if not selected:
        if not sys.stdin.isatty():
            raise CanError("请指定接口或 --all；仅查看请使用 --list。")
        answer = input("输入序号或接口名（多个用空格分隔），all 启用全部，q 退出：").strip()
        if answer.lower() in ("", "q", "quit"):
            print("已退出。")
            return 0
        if answer.lower() == "all":
            selected = list(links)
        else:
            names = list(links)
            selected = []
            for item in answer.split():
                if item.isdigit():
                    index = int(item)
                    if not 1 <= index <= len(names):
                        raise CanError("接口序号超出范围：" + item)
                    item = names[index - 1]
                selected.append(item)
    selected = list(dict.fromkeys(selected))
    network_selection = selected
    if not args.can_only:
        required = prepare_motors(selected, args.bitrate, args.dry_run)
        network_selection = list(dict.fromkeys(selected + required))
        for name in network_selection:
            if name not in links:
                raise CanError("未发现 CAN 接口：{}。用 --list 查看当前接口。".format(name))
            if not args.reset_only:
                needs_enable(links[name], args.bitrate)
        if not args.keep_state:
            command = motor_command(selected, check=True)
            command[command.index("--check")] = "--reset-state"
            if args.dry_run:
                print("预览软件状态归档重置：" + shlex.join(command))
            else:
                reset_result = json.loads(run(command, capture=True))
                if reset_result.get("archive_path"):
                    print("旧软件状态{}；原账本：{}".format(
                        "已归档重置" if reset_result.get("reset_performed") else "已在本次启动处理，未重复重置",
                        reset_result["archive_path"]), flush=True)
                else:
                    print("没有需要重置的旧任务软件状态。", flush=True)
        if args.reset_only:
            return 0
        # A no-op on an already-up network must reach the TX-blocked state
        # reader even after a startup fault. Keep the old owner/fault gate
        # before bringing a down interface up, which can affect the bus.
        if not args.dry_run and any(needs_enable(links[name], args.bitrate) for name in network_selection):
            command = motor_command(selected, check=True)
            command[command.index("--check")] = "--check-startup"
            run(command, capture=True)
        print("本次将检查 CAN 和所选接口对应的关节电机；已使能则跳过：" + "、".join(selected), flush=True)
        extra = [name for name in required if name not in selected]
        if extra:
            print("为读取另一臂状态，同时开启其 CAN 通信（不使能该臂电机）：" + "、".join(extra), flush=True)
    enable(ip, network_selection, links, args.bitrate, args.dry_run)
    if not args.can_only:
        command = motor_command(selected)
        if args.dry_run:
            print("预览电机使能：" + shlex.join(command))
        else:
            run(command)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (CanError, OSError, ValueError) as exc:
        print("错误：{}\n已停止后续操作；已完成的接口配置保留，请用 --list 查看。".format(exc), file=sys.stderr)
        sys.exit(1)
    except (KeyboardInterrupt, EOFError):
        print("\n操作已中断；请用 --list 查看当前接口状态。", file=sys.stderr)
        sys.exit(130)
