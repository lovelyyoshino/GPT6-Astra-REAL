---
name: piper-task-pipeline
description: 在 Piper 交接包中通过 Codex 规划、执行或复盘桌面任务，按任务 pipeline 调用当前有界操作；覆盖单臂、双臂和主臂执行辅臂观察的能力分流与预算退出。
---

# L3 任务入口

从项目根目录的 `AGENTS.md` 获取本项目的 Codex 使用方式和实机边界。固定本轮 task、mode、角色、初态、目标及总预算；已有明确指令直接采用，不重复采访。

在 `projects/piper_right_pick_demo` 下用纯离线命令取得契约。首次读取完整单任务，冻结初态、目标、约束及步骤：

```bash
PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --task pen
```

其他任务用 `--task <任务ID>`，双臂加 `--mode dual_arm`，辅助观察加 `--mode worker_with_observer --worker-arm left`。仅不清楚任务 ID 时用 `--catalog`；后续只传宿主账本的 `current()` 与剩余预算。`--compact` 仅展示新账本的初始阶段，不保存状态，反复启动它不能代替续跑。

跨 Codex 工具调用维护同一离线任务时，用持久化入口；同一轮始终沿用 store 与 run-id：

```bash
PYTHONPATH=src python3 -m right_pick.fast_task_session --store runs/task_sessions.sqlite init --run-id pen-001 --task pen
PYTHONPATH=src python3 -m right_pick.fast_task_session --store runs/task_sessions.sqlite current --run-id pen-001
PYTHONPATH=src python3 -m right_pick.fast_task_session --store runs/task_sessions.sqlite current --run-id pen-001 --operation
```

首次用 `contract --run-id pen-001` 核对冻结的完整任务。普通 `current` 保持短包；需要当前 L2 的前提、证据、周期和 L1 名称时加 `--operation`，不重新查询全任务或手工猜操作名。后续用 `record --run-id pen-001 --revision <current中的revision> --event-id <本次事件ID> --receipt <宿主JSON文件>` 记录事实；回执也必须包含相同 run_id。`observer` 使用相同参数记录最新保持回执并进入观察分支，`end` 记录明确退出原因。当前阶段引用过期会拒绝，重复同一 event-id 不再次推进；重复 init、进程重启均不重置总预算。不要把模型自述伪装成宿主证据；该入口只做离线记账，没有设备发送能力。

`status=progress` 只有宿主提供绑定当前 `observation_id` 的数值 `progress_measurement`（`metric`、`unit`、`before`、`after`、`observation_id`），且同阶段同指标沿原方向连续变化，才清零无进展计数。空、重复或倒退的进展主张消耗无进展预算；模型自述和设备到位不能充当物体进展测量。

调用 [L2 操作入口](../piper-manipulation/SKILL.md)，只传当前 stage、arm、needs、expect 和剩余预算。`current --operation` 返回的 L1 名称与 [原子 Codex 技能索引](../../../ATOMIC_SKILLS.md)一一对应；只读取本轮实际需要的 `$piper-atom-*`，同名契约仍由宿主代码查询和执行。双臂/观察模式按需读 [角色协同](../piper-arm-coordination/SKILL.md)。这些层是宿主调用关系，不是层层新建模型会话。

当前单右臂 `pen` 已有 phase runner 与 Codex 后端；其他 recipe 及协同模式标为 `offline_contract_only`。先检查实现状态；不能直接把离线账本接上硬件。

用户要求执行时，使用同一有效控制宿主与当前资格；模型入口显式 `--model codex` 和 Codex 配置，沿用现有登录。当前现场准入缺失时一次报告具体缺口并结束该执行请求；用户要求修复缺口时进入对应有限诊断流程。

结束分别报告任务、回位、发送/模型调用、耗时、人工参与与 `termination_reason`。`offline_contract_completed` 只说明账本通过，不能写成实机成功。详见 [层次和任务映射](../../../projects/piper_right_pick_demo/docs/ATOMIC_SKILLS_AND_DUAL_COORDINATION.md)。
