---
name: piper-atom-set-gripper-once
description: Piper 已准入的单次夹爪目标动作类型，区分夹爪到位和抓取成功。
---

# set_gripper_once

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic set_gripper_once`（项目目录下）。这是 `dispatch_once` 内的动作类型，不额外发送。闭爪回执只证明夹爪事务，抓取须在新图和有限试提后验证。
