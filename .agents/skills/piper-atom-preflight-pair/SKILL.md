---
name: piper-atom-preflight-pair
description: Piper 双任务臂或观察臂任务启动前联合核对两臂与唯一控制宿主。
---

# preflight_pair

只在 L3 双臂或 worker_with_observer 启动时调用。查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic preflight_pair`（项目目录下）。两侧和共享宿主须同时具备当前资格；当前仅为离线契约，不能打开双臂实机执行。
