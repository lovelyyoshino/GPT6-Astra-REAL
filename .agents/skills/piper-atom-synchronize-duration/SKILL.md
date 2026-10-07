---
name: piper-atom-synchronize-duration
description: 为 Piper 双臂已准入的有限段检查共同 duration 与时序边界。
---

# synchronize_duration

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic synchronize_duration`（项目目录下）。只校验当前宿主计划的共同有限时长；near-time 顺序发送不是固件同步或原子事务。当前仅离线契约。
