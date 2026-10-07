---
name: piper-atom-plan-swept-corridor
description: 核对 Piper 整臂、末端附件和场景通道的路径准入证据。
---

# plan_swept_corridor

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic plan_swept_corridor`（项目目录下）。端点步长不证明整臂路径；当前函数只校验宿主提供的通道结果，尚无已验证碰撞规划器。缺少当前整臂路径证据时拒绝派发。
