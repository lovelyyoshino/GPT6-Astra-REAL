---
name: piper-atom-set-gripper-once
description: Piper 已准入的单次夹爪目标动作类型，区分夹爪到位和抓取成功。
---

# set_gripper_once

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic set_gripper_once`（项目目录下）。这是 `dispatch_once` 内的动作类型，不额外发送。闭爪回执只证明夹爪事务；抓取按当前支撑方式在新图中验证。普通拾取可采用准入后的有限试提，在座插头/固定插排采用支持下的接触证据，不能一律试提。

需要建立接触时，按 [接触与微调](../piper-manipulation/references/bounded-contact.md) 先发宿主允许的一次有界夹爪目标，再依据新反馈微调；不把“尚未夹住”作为拒绝首次闭爪的理由。pair 的 `grip_supported` 夹爪路径可返回接触候选；可显式同爪有界 `release_retreat` 开爪。需另一臂接续时，probe 带物体身份，动作后新 RGB 与适配器新 trace 经 `robot_pair_retain_grasp` 建立零 TX 静态保持，只允许另一空臂接近、对齐或试夹，不能继续拉动或加力。普通开口到位路径仍按原到位标准；候选和到位均不能被模型改写成抓取成功。
