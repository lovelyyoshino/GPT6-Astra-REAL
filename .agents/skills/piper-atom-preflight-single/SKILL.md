---
name: piper-atom-preflight-single
description: Piper 单臂任务启动前核对当前机械臂、控制宿主、会话和现场准入。
---

# preflight_single

只在 L3 单臂 pipeline 启动时调用一次。查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic preflight_single`（项目目录下）。沿用当前有效设备与会话证据，不用历史快照替代当前状态。失效宿主、待机或未使能按 [在线接续分流](../piper-task-pipeline/references/online-readiness.md) 复用已有准备入口，完成后继续本任务，不反复 preflight。必要项确实无法补齐才返回具体 unavailable；准备结果不自动授予运动或保持资格。
