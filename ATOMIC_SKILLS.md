# GPT6-Astra-REAL 原子 Codex 技能索引

项目技能位于隐藏目录 [`.agents/skills`](.agents/skills)。从 `/home/agilex/GPT6-Astra-REAL` 启动 Codex，按名称调用 `$piper-task-pipeline`；需要某个 L1 原子时再读对应 `$piper-atom-*`。本页提供可见入口。

从任意终端目录启动时，指定当前项目：

```bash
codex -C /home/agilex/GPT6-Astra-REAL
```

进入后用 `/skills` 查找 `piper-task-pipeline`，或输入 `$piper-atom-move-eef-once` 检查单项技能。旧会话的工具命令中 `cd` 不等于重新发现项目技能；技能缺失时从本目录重新进入。无需复制到用户级目录，沿用当前 Codex 登录。

调用顺序是 **L3 任务 recipe → L2 当前有界操作 → L1 当前原子 → L0 Piper 适配器**。L3 与 L2 决定本轮需要什么；L1 给出一项工作的边界并查询同名 Python 契约，宿主负责校验、计数、派发与回执。层级不意味着每层调用一次模型。首次读当前任务，后续只读持久 session 的当前操作和所需原子，不一次加载全部 25 个技能。复杂 recipe 复用相同原子，保持有限步骤和共享预算。

根目录 `./astra catalog` 查看任务和组合，`./astra plan --recipe tasks/sort_two_objects.json` 查看组合示例。`place_on_support` 和 `articulated_rotate` 分别表达目标支撑放置与关节物体旋转，其合同不会增加底层执行权限。

| L1 调用时机 | 可单独发现的 Codex 技能 |
| --- | --- |
| 启动 | [preflight_single](.agents/skills/piper-atom-preflight-single/SKILL.md)、[preflight_pair](.agents/skills/piper-atom-preflight-pair/SKILL.md)、[renew_session](.agents/skills/piper-atom-renew-session/SKILL.md) |
| 新场景 | [observe_scene](.agents/skills/piper-atom-observe-scene/SKILL.md)、[check_fresh_observation](.agents/skills/piper-atom-check-fresh-observation/SKILL.md) |
| 单臂决策与准入 | [decide_one](.agents/skills/piper-atom-decide-one/SKILL.md)、[admit_action](.agents/skills/piper-atom-admit-action/SKILL.md)、[plan_swept_corridor](.agents/skills/piper-atom-plan-swept-corridor/SKILL.md) |
| 单臂一次动作 | [dispatch_once](.agents/skills/piper-atom-dispatch-once/SKILL.md)、[move_eef_once](.agents/skills/piper-atom-move-eef-once/SKILL.md)、[set_gripper_once](.agents/skills/piper-atom-set-gripper-once/SKILL.md)、[read_receipt](.agents/skills/piper-atom-read-receipt/SKILL.md) |
| 物体与结束证据 | [verify_visual](.agents/skills/piper-atom-verify-visual/SKILL.md)、[verify_task_evidence](.agents/skills/piper-atom-verify-task-evidence/SKILL.md)、[sample_stability](.agents/skills/piper-atom-sample-stability/SKILL.md)、[verify_return](.agents/skills/piper-atom-verify-return/SKILL.md) |
| 双任务臂 | [decide_pair](.agents/skills/piper-atom-decide-pair/SKILL.md)、[coordinate_pair](.agents/skills/piper-atom-coordinate-pair/SKILL.md)、[synchronize_duration](.agents/skills/piper-atom-synchronize-duration/SKILL.md)、[prepare_held_side](.agents/skills/piper-atom-prepare-held-side/SKILL.md)、[dispatch_pair_once](.agents/skills/piper-atom-dispatch-pair-once/SKILL.md)、[latch_pair_fault](.agents/skills/piper-atom-latch-pair-fault/SKILL.md) |
| 一臂工作、一臂观察 | [coordinate_observer](.agents/skills/piper-atom-coordinate-observer/SKILL.md)、[freeze_observer](.agents/skills/piper-atom-freeze-observer/SKILL.md)、[handoff_hold](.agents/skills/piper-atom-handoff-hold/SKILL.md) |

每个链接是一份真实 `SKILL.md`，与 `right_pick.fast_pipeline.ATOMIC_SKILLS` 中的一个同名契约一一对应。查询契约的命令从 `projects/piper_right_pick_demo` 执行，例如 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic move_eef_once`。查询不会启动模型、ROS 或机械臂。

**实现状态**：既有物理执行路径是单右臂 `pen` runner，另保留历史专项入口。历史 fast 资格曾撤回，本次没有重建通用真机资格。25 个技能文件和 Python 契约不等于 25 个已验证的物理动作；通用 recipe、复杂组合、双任务臂与观察臂仍是离线编排/校验能力。技能、经验和任务 JSON 均不能替代当前执行资格。
