---
name: piper-atom-preflight-single
description: Piper 单臂任务启动前核对当前机械臂、控制宿主、会话和现场准入。
---

# preflight_single

只在 L3 单臂 pipeline 启动时调用一次。查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic preflight_single`（项目目录下）。使用当前设备与会话证据；不复用历史资格。缺项返回具体 unavailable，禁止进入派发。
