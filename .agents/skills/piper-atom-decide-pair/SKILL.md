---
name: piper-atom-decide-pair
description: Piper 双任务臂从同一场景版本提出有界的左右臂联合提案。
---

# decide_pair

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic decide_pair`（项目目录下）。一次决策绑定同一 observation_id；本批 recipe 每周期至多一侧移动，另一侧为带回执的保持。当前仅离线契约，不调用双臂硬件。
