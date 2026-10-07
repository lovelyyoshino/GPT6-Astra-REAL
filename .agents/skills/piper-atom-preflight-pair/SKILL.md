---
name: piper-atom-preflight-pair
description: Piper 双任务臂或观察臂任务启动前联合核对两臂与唯一控制宿主。
---

# preflight_pair

只在 L3 双臂或 worker_with_observer 启动时调用。查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic preflight_pair`（项目目录下）。沿用两侧与共享宿主的当前有效资格。失效宿主、待机或未使能按 [在线接续分流](../piper-task-pipeline/references/online-readiness.md) 处理，只补实际缺少的准备步骤，不反复 preflight。该原子仍仅为离线契约，没有硬件派发能力；现成在线入口是否覆盖本任务独立判断。双臂任务一次只动一臂，另一臂须有当前独立保持回执；启动成功和读取两臂状态均不等于已获得此能力。
