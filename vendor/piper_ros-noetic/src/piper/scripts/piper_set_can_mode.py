#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author        pony
@date          2026-09-30
@version       v1.0.0
@last_modified 2026-09-30
@changelog     - v1.0.0 (2026-09-30): 新增 Piper CAN 控制模式切换脚本
"""

import argparse
import os
import time

from piper_sdk import C_PiperInterface


def main() -> None:
    parser = argparse.ArgumentParser(description="使能 Piper 并切换到 CAN 控制模式")
    parser.add_argument("--can", help="CAN 接口名称，也可输入 arm1/arm2")
    parser.add_argument("--speed", type=int, default=50, help="运动速度百分比 0-100")
    args = parser.parse_args()
    can_name = args.can or input("请输入 CAN 名称（arm1/arm2/can_arm1/can_arm2）：").strip()
    can_name = {"arm1": "can_arm1", "arm2": "can_arm2", "cam_arm0": "can_arm0", "cam_arm1": "can_arm1", "cam_arm2": "can_arm2"}.get(can_name, can_name)
    if not os.path.exists(f"/sys/class/net/{can_name}"):
        raise SystemExit(f"CAN 接口不存在：{can_name}。可用接口请运行：ip -br link show type can")
    speed = max(0, min(args.speed, 100))

    piper = C_PiperInterface(can_name=can_name)
    piper.ConnectPort()
    time.sleep(0.8)
    # 示教按钮松开后仍可能保留示教控制状态，0x02 明确结束示教记录。
    piper.MotionCtrl_1(0x00, 0x00, 0x02)
    time.sleep(0.3)
    piper.EnableArm(7)
    # ctrl_mode=1 表示 CAN 控制，move_mode=1 表示 MOVE_J。
    for _ in range(10):
        piper.MotionCtrl_2(0x01, 0x01, speed, 0x00)
        time.sleep(0.1)
    time.sleep(1.0)

    status = piper.GetArmStatus().arm_status
    print(status)
    print("CAN:", can_name)
    print("enable:", piper.GetArmEnableStatus())
    if "CAN_CTRL" not in str(status.ctrl_mode):
        print("警告：当前仍未进入 CAN_CTRL，请确认机械臂已重启且不在示教模式。")


if __name__ == "__main__":
    main()
