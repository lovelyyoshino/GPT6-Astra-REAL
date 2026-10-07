---
name: piper-atom-dispatch-once
description: Piper 宿主对已准入的单臂动作至多发送一次并记录发送结果。
---

# dispatch_once

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic dispatch_once`（项目目录下）。只有 L0 当前资格和 admission_receipt 允许发送。`move_eef_once` 或 `set_gripper_once` 是本次 dispatch 的动作类型，不额外再发。结果不确定即锁存，不自动重试。
