---
name: piper-atom-freeze-observer
description: Piper 观察臂控制宿主冻结与工作臂保持交接的离线核对。
---

# freeze_observer

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic freeze_observer`（项目目录下）。核对观察宿主冻结和工作臂 hold 未变；任一竞态锁存双臂。当前没有接通的 Piper 双臂宿主交接。
