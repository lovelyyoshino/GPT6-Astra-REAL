---
name: piper-atom-dispatch-pair-once
description: Piper 双臂共享 barrier 对联合准入的成对目标只做一次派发尝试。
---

# dispatch_pair_once

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic dispatch_pair_once`（项目目录下）。当前仅定义离线契约；不得用两个单臂 CLI 拼接。任一侧部分发送或结果未知即锁存整对，不补发另一侧。
