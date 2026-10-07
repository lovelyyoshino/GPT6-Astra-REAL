---
name: piper-atom-handoff-hold
description: Piper 静止保持目标跨控制会话交接前核对目标与宿主一致性。
---

# handoff_hold

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic handoff_hold`（项目目录下）。只在同一不变保持目标有当前回执时允许交接；这是离线核对，不提供 powered hold 或重新授权。失败时不释放原保持。
