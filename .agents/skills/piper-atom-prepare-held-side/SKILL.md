---
name: piper-atom-prepare-held-side
description: Piper 一臂动作前核对另一臂静止保持的独立当前回执。
---

# prepare_held_side

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic prepare_held_side`（项目目录下）。null/held 侧须有同场景、同宿主的独立保持证据；Piper 当前尚无已验证 powered hold，不能把 timeout 或不发命令当作保持资格。
