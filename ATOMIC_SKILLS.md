# Piper 原子 Codex 技能索引

项目技能实际位于隐藏目录 [`.agents/skills`](.agents/skills)。从本交接包根目录启动 Codex 后，可按名称调用 `$piper-task-pipeline`；需要某个 L1 原子时再调用对应 `$piper-atom-*`。在 Finder 中没有看到它们，是因为 `.agents` 目录默认隐藏；本页提供可见入口。

在终端从任意目录启动时，明确指定本交接包为 Codex 工作目录：

```bash
codex -C /Users/pony.ai/Documents/文档/Piper_SingleArm_Handoff
```

进入后用 `/skills` 查找 `piper-task-pipeline`，或输入 `$piper-atom-move-eef-once` 检查单项技能。若会话已经从上一级 `…/文档` 启动，仅在工具命令中 `cd` 进本目录不会改变该会话启动时的技能发现范围；请从本目录新开 Codex 会话。Codex 从启动目录向上扫描 `.agents/skills`，不向下搜索子目录。Finder 可用 `Command+Shift+.` 显示隐藏目录。这里不需要安装到用户级技能目录，也不需要 API key。

调用顺序是 **L3 单项任务 → L2 当前有界操作 → L1 当前原子 → L0 Piper 适配器**。L3 与 L2 决定本轮需要什么；L1 技能给 Codex 一项工作的边界并查询同名 Python 契约，宿主负责校验、计数、派发和回执。层级不意味着每层调用一次模型。首次读当前任务，后续只读 `fast_task_session current --operation` 给出的当前操作和所需原子；不要一次加载全部 25 个技能。

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

**实现状态**：当前只有单右臂 `pen` 的既有物理执行源码，现场连续执行准入已撤回。25 个 Codex 技能和 Python 契约不等于 25 个实机动作。双任务臂、观察臂和通用任务是离线编排与校验能力，不能直接物理执行。
