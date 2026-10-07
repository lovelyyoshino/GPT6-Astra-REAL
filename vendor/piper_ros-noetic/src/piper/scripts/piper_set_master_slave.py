#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author        pony
@date          2026-09-30
@version       v1.0.0
@last_modified 2026-09-30
@changelog     - v1.0.0 (2026-09-30): 新增 Piper 主从模式切换脚本
"""

import argparse
import os
import time

from piper_sdk import C_PiperInterface


def main() -> None:
    parser = argparse.ArgumentParser(description="设置 Piper 主臂或从臂模式")
    parser.add_argument("--can", help="CAN 接口名称，也可输入 arm1/arm2")
    parser.add_argument(
        "--mode",
        choices=("master", "slave", "主臂", "从臂", "示教输入", "运动输出"),
        help="master=示教输入臂，slave=运动输出臂",
    )
    args = parser.parse_args()
    can_name = args.can or input("请输入 CAN 名称（arm1/arm2/can_arm1/can_arm2）：").strip()
    mode = args.mode or input("请输入模式（master/slave/主臂/从臂）：").strip()
    can_name = {"arm1": "can_arm1", "arm2": "can_arm2", "cam_arm0": "can_arm0", "cam_arm1": "can_arm1", "cam_arm2": "can_arm2"}.get(can_name, can_name)
    if not os.path.exists(f"/sys/class/net/{can_name}"):
        raise SystemExit(f"CAN 接口不存在：{can_name}。可用接口请运行：ip -br link show type can")

    # 0xFA 是示教输入臂，0xFC 是运动输出臂；切换后通常需要重启机械臂。
    linkage_config = 0xFA if mode in ("master", "主臂", "示教输入") else 0xFC
    piper = C_PiperInterface(can_name=can_name)
    piper.ConnectPort()
    time.sleep(0.5)
    piper.MasterSlaveConfig(linkage_config, 0x00, 0x00, 0x00)
    print(f"已发送 {mode} 模式配置到 {can_name} (0x{linkage_config:02X})")
    print("请给机械臂断电重启后，再运行 CAN 控制模式脚本。")


if __name__ == "__main__":
    main()
