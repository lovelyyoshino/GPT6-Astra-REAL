# Piper 分层原子技能与任务 Pipeline

用户入口是进入本项目的 Codex，沿用现有登录和 CLI。任务结构借鉴 `GPT6-ARX5/skills/arx-r5-tabletop-tasks`；闭环、回执和有限恢复借鉴两份 Astra LLM-as-Policy 文档。参考资料中的任务难度 L1/L2 与本文的软件层次 L0–L3 无关。

ARX5 原目录包含一个总 skill、18 个任务指南、共用执行和证据规则，并不是可直接移植的原子执行库。本包将其拆成 **18 个任务 recipe、20 个组合操作、25 个原子契约**，并为 25 个原子各提供项目内独立 Codex `SKILL.md`；[可见索引](../../../ATOMIC_SKILLS.md)解释隐藏路径与按需调用。契约与 Piper 实机能力分开标注。

## 1. 调用层次

```text
Codex 外层 → L3 单项任务：初态、目标、角色、阶段、总预算
            → L2 当前操作：前提、有限循环、完成证据、局部预算
              → L1 原子：新观测 → 一次提案 → 准入 → 一次发送 → 回执
                → L0 Piper ROS / 相机 / 资格 / 实测状态
                ← 下一张新图与执行结果 → 推进或终止
```

| 层 | 代码与 Codex 入口 | 责任与实际状态 |
| --- | --- | --- |
| L3 | `fast_task_pipeline.py:TASK_RECIPES / BoundedTaskPipeline`，`fast_task_session.py`；[任务入口](../../../.agents/skills/piper-task-pipeline/SKILL.md) | 18 项离线任务编排，持久化宿主事件账本，不发送动作。 |
| L2 | `COMPOSITIONS / composition_contract()`；[操作入口](../../../.agents/skills/piper-manipulation/SKILL.md) | 20 项有界操作显式引用原子接口；通用操作未接实机。现有夹笔 phase runner 通过 `pen_phase_contract()` 对应部分阶段。 |
| L1 | `fast_pipeline.py`；[原子总路由](../../../.agents/skills/piper-atomic-runtime/SKILL.md)、[25 个单项 Codex 入口](../../../ATOMIC_SKILLS.md) | 每项独立查询输入、输出、副作用和失败契约；纯校验函数不代表已完成硬件能力。 |
| 协同 | `pipeline_contract / validate_pair_*`；[协同入口](../../../.agents/skills/piper-arm-coordination/SKILL.md) | 共同场景、held-side、独立回执和整对故障语义，当前离线。 |
| L0 | `fast_ros / fast_observation / fast_safety / fast_qualification` | 已有单右臂 ROS 与三相机适配源码；当前现场准入撤回。 |

四个层级入口与 25 个轻量原子 SKILL.md 按需加载。原子接口放在代码目录中逐项查询；Codex 只读当前操作需要的入口，避免每轮加载整个目录。模型只在 decide 阶段选择动作；校验、计数、发送和回执由宿主执行，不为每层新建模型会话。

## 2. 在 Codex 中使用

从交接包根目录打开 Codex，先读根 `AGENTS.md`。在 `projects/piper_right_pick_demo` 下运行：

```bash
# 首次完整读取单任务，冻结目标、约束和步骤
PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --task pen
# 按需读取当前操作与原子，不是发送动作
PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --composition grip_test
PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic move_eef_once
# 双任务臂 / 左臂工作、右臂观察的离线契约
PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --task cups --mode dual_arm
PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --task pen --mode worker_with_observer --worker-arm left
```

`--catalog` 用于一次性发现名称。`--compact` 只展示新建账本的首个阶段，不保存运行状态；后续阶段读取同一宿主的 `current()`，不能反复运行 CLI 代替续跑或重置预算。

`BoundedTaskPipeline.record_cycle()` 接收宿主的 task、stage、arm、observation_id、时间、model_called、status、prerequisites 和 evidence，检查绑定、阶段、观测、预算及故障。它不能认证调用方提供的物理事实。终态是 `offline_contract_completed`，`execution_available=False`。

跨命令调用使用持久化入口，不需要让 Codex 将全部事件重复写回提示词：

```bash
PYTHONPATH=src python3 -m right_pick.fast_task_session --store runs/task_sessions.sqlite init --run-id pen-001 --task pen
PYTHONPATH=src python3 -m right_pick.fast_task_session --store runs/task_sessions.sqlite contract --run-id pen-001
PYTHONPATH=src python3 -m right_pick.fast_task_session --store runs/task_sessions.sqlite current --run-id pen-001
PYTHONPATH=src python3 -m right_pick.fast_task_session --store runs/task_sessions.sqlite current --run-id pen-001 --operation
```

`init` 只创建一次，同一 run-id 再调用会返回既有阶段与剩余预算；任务、模式、角色及预算不可替换。`contract` 首次读取完整冻结任务，`current` 后续只返回当前阶段、revision、计数和剩余时间；`current --operation` 按需展开绑定该阶段的 L2 契约及 L1 名称，避免手工查错操作。单臂/双臂/观察模式沿用上面的 `--mode` 和 `--worker-arm` 参数。

`record --run-id pen-001 --revision N --event-id cycle-N --receipt host-receipt.json` 接收该轮宿主回执，其中还必须有 `run_id=pen-001`。`observer` 用同样参数接收当前主臂保持回执，完成观察周期后恢复原工作阶段。`end --run-id pen-001 --revision N --event-id end-N --reason '具体原因'` 只终止离线账本，不执行机器人停止。

`status=progress` 需要宿主核验的数值变化 `progress_measurement={"metric":"object_displacement","unit":"mm","before":0,"after":2,"observation_id":"frame-2"}`；同阶段同指标后续测量须从前次 `after` 连续推进。无测量、数值不变、旧测量重贴或倒退均计作无进展，达到预算即退出。此结构只能约束账本，不能自行认证物理测量来源。

SQLite 事务防止两个 Codex 命令同时推进相同 revision；重复同一 event-id 和相同内容是幂等读取，换内容会拒绝。重启后按事件重建阶段，累计时间仍从原始创建时刻计算。过期、故障和已结束任务不能重开；系统时钟倒退时锁住账本。契约发生代码变更时拒绝静默复用旧运行，需要显式迁移。账本不是硬件互斥锁或物理事实认证，不承接当前 pen 实机 runner 的续跑。

现有夹笔执行入口仍为 `right_pick.cli --mode astra_fast_closed_loop`；显式使用 `--model codex` 与 `protocol=codex_cli` 配置，无需 API key。默认 example 已改为 Codex。旧 Responses 后端仅保留兼容，当前使用流程不要求配置它。CLI 拒绝后端与配置协议不匹配。

外层 Codex 读取技能、调用宿主；`fast_codex.py` 内层是隔离、禁工具的单步 JSON 决策，不读项目手册或重做整项任务规划。已有有效宿主和准入时直接进入任务；缺能力时一次报告具体缺口。不要循环 prepare 或另开控制宿主。

## 3. L1 原子接口与 ARX5 映射

| 组 | 原子接口 | 实现边界 |
| --- | --- | --- |
| 启动与观测 | `preflight_single`, `preflight_pair`, `observe_scene`, `check_fresh_observation` | 单右臂已接准入和新图；双臂检查输入契约。新图核对身份、帧号、时间与 skew。 |
| 提案与准入 | `decide_one`, `decide_pair`, `admit_action`, `plan_swept_corridor` | Codex 单提案已接入；整臂通道函数仅校验宿主结果，尚非碰撞/路径规划器。 |
| 动作与回执 | `move_eef_once`, `set_gripper_once`, `dispatch_once`, `dispatch_pair_once`, `read_receipt` | 前两个是 dispatch 内的动作种类，不能各再发一次。单右臂支持末端或夹爪；双臂 barrier 为契约。 |
| 协同时序 | `synchronize_duration`, `prepare_held_side`, `coordinate_pair`, `coordinate_observer` | 共用场景和 duration；null/held 要独立回执；观察侧禁止任务接触。 |
| 会话与故障 | `renew_session`, `latch_pair_fault`, `freeze_observer`, `handoff_hold` | 纯校验与锁存语义；未实现 Piper powered hold 或在线交接；续期不重置总预算。 |
| 结束证据 | `verify_visual`, `verify_task_evidence`, `sample_stability`, `verify_return` | 区分模型报告与测量；稳定用新样本；回位核对本轮参考、完整末端位姿、实测与指令关节。 |

ARX5 共用流程中的宿主检查映射 preflight，续段映射 renew，观察交接映射 freeze/handoff，独立稳定与回位映射 sample_stability/verify_return。Piper 不复制 ARX5 的 CAN、TCP、基座变换、powered hold、stop 或历史动作。

原子性指有限接口和至多一次副作用，不代表固件原子事务。发送结果不确定即锁存，禁止重发。关闭客户端或失能不能证明安全停止。

## 4. L2 有界操作

每个操作共用观测、准入、回执和故障循环，不复制整套提示词。`composition_contract()` 返回 primitives、requirements、evidence、max_cycles 和 guards。

| 操作 | L1 primitive | 完成证据 |
| --- | --- | --- |
| `inspect` | observe_scene | 初态冻结，本轮回位参考保存 |
| `approach`, `align` | move_eef_once | 接近通道、物体对齐可见 |
| `grip_test` | set_gripper_once, move_eef_once | 有限试提后物体随爪、离开支撑 |
| `grip_supported` | set_gripper_once | 两侧接触、原支撑保持 |
| `validate_preheld` | observe_scene | 预持成立，不计自主拾取 |
| `transport` | move_eef_once | 持物保持、目的地可见 |
| `lower_to_support`, `hang_supported` | move_eef_once | 指定支撑/挂钩接住物体 |
| `insert_segment`, `rotate_segment`, `extract_segment` | move_eef_once | 物体轴向、相对旋转/退丝/分离 |
| `pour_segment`, `wipe_segment`, `sweep_segment`, `push_segment` | move_eef_once | 接收、接触滑动、全部目标入区、物体位移 |
| `release_retreat` | set_gripper_once, move_eef_once | 支撑成立、夹爪离开 |
| `stable_verify` | sample_stability | 两次新观测相隔至少 2 秒，独立稳定 |
| `return_reference` | move_eef_once, verify_return | 本轮不可变参考与静止状态吻合 |
| `observer_reposition` | prepare_held_side, move_eef_once | 主臂保持，只改善视角，新双臂观测 |

含多个 primitive 的操作分周期执行，每周期至多一次发送。接触操作还要求适配器真实具备对应接触步能力，名称和配置不构成该能力。

## 5. 18 项任务映射

下面省略共同的 inspect、对齐、释放/稳定、回位步骤；完整顺序及约束以 `--task` 为准。

| 任务 ID | 核心操作 / 角色 | 保留的变体与证据 |
| --- | --- | --- |
| cups | 左放杯并回位 → 右套叠 | 双任务臂；左整臂退出右侧通道 |
| pen | 试抓 → 搬运 → 插入 | 不替换细笔或指定笔筒 |
| charger | 试抓 → 搬运 → 插入 | 指定断电插座、机械插入 |
| charger-insert-only | 支撑夹持 → 插入 | 人工预对齐，不试提或计自主拾取 |
| flower | 预持核验 → 搬运 → 插入 | 预持、接收器和花茎朝向单列 |
| hat | 试抓 → 搬运 → 挂靠 | 指定真实挂钩，帽架稳定 |
| pen-uncapping | 左持帽 / 右持笔 → 分离 | 双任务臂；人工松帽与预持单列 |
| pearl-pouring | 试抓 → 搬运 → 有限倾倒 | 洒落、残留、接收数量标准冻结 |
| orange-pick-place | 试提 → 放回支撑 | 橘瓣/整果和抬升范围明确 |
| drawer-push-pull | 支撑夹持 → 拉出 → 推回 | 抽屉相对位移，恢复初始开度 |
| jelly-pick-place | 试提 → 放回支撑 | 软物体不靠加力补偿滑移 |
| bottle-unscrewing | 左持瓶 / 右旋盖 → 分离 | 双任务臂；夹持不证明抗扭 |
| fixed-bottle-unscrewing | 工装固定 → 工作臂旋盖 | 工装抗扭；观察臂不能固定瓶身 |
| bolt-screwing | 右持螺栓 / 左旋螺母 | 三圈相对旋转与进给，非腕角 |
| book-extraction | 支撑夹持 → 抽取 | 只计初态之后的新增书本位移 |
| blackboard-wiping | 核验预持工具 → 擦拭 | 擦拭动作与擦除效果分别定义 |
| blue-blocks-sweeping | 预置扫帚 → 扫入 → 撤刷 | 全部目标入区，观察臂不扶簸箕 |
| blue-block-triangle-push | 空爪边缘推 → 撤离 | 不抓放，不增加居中要求 |

cups、拔帽、双臂旋盖、旋螺母必须用 dual_arm。其余可声明单工作臂或 view-only 辅臂；这是契约角色支持，尚非实机能力声明。

## 6. 预算与减少无效调用

现有单右臂 fast 默认：32 步、24 模型调用、每阶段 4 步、拒绝重试 2、允许一个 RECOVERY 阶段、连续 observe(unknown) 2 次、wall-clock 900 秒。`--execution` 必填，避免无意进入 prepare。已修正恢复上限的边界错误：第一次进入 RECOVERY 可以用新观测纠正未发送的提案；第二次进入才停止，其他总预算不重置。发送结果不确定仍立即锁存，不能借恢复重发。到期禁止新 dispatch，已接受目标仍服从驱动超时合同；900 秒不保证硬件立即停止。

通用离线账本默认：128 周期、64 模型调用、900 秒、连续无进展 2、拒绝 1、观察换位 2 次；各操作另有限额。progress 必须有宿主核验的新数值状态变化，不能把重复静态观测或“准备好了”计为推进。

首次加载单任务全文；之后只传当前 phase、局部操作、有限动作预算、上一结果、短记忆和选定 RGB。完整手册及原子目录不进入每次 build_codex_packet。宿主检查无需额外模型调用。

历史零动作一轮耗时 101.141 秒，11 次模型接口累计 97.937 秒，说明问题同时来自无进展决策和接口等待。离线测试证明预算退出与输入收敛，尚不能证明实机 token 降幅或完成时间。

## 7. 双臂与辅助观察审查

双臂循环：共同新场景 → decide_pair → 联合准入 → shared barrier → 两侧独立回执 → 新场景。本批 recipe 一次只动一臂；静止侧也须提供同场景的独立保持回执。一侧部分发送或结果未知即锁存整对，不补发。

辅助观察：最新主臂 hold → 观察臂有限换位 → 独立回执 → 新双臂图像 → 恢复原工作阶段。观察臂不能固定、支撑或抓取目标；拔帽、扶书或固定瓶身属于第二个任务角色。

现有 `projects/piperx_cloth_demo/robot_tools/execution.py` paired 是校验后顺序 near-time send 加完成 barrier，不是原子提交或硬同步。`single_supervised_actions.py` 的被动臂 TX-block 必须保留，旧链不能直接拼接成新双臂执行器。

| 待接能力 | 当前缺口 | 可复用离线接口 |
| --- | --- | --- |
| 共同控制宿主 | fast 仅绑定右臂，无双臂 ROS 资格/执行实例 | validate_pair_admission / validate_pair_receipt |
| 保持与交接 | Piper timeout 为有限目标完成，不是已验证 powered hold | prepare_held_side / validate_hold_handoff |
| 整臂路径与时序 | 端点限制不证明连杆、腕相机和线缆净空 | validate_swept_corridors / synchronize_duration |
| 通用操作执行 | L2/L3 未接具备接触能力的物理适配器 | composition_contract / BoundedTaskPipeline |
| 稳定与回位 | 通用账本含双样本与回位；既有夹笔 runner 尚无自动回位或两秒双样本验证 | sample_stability / validate_return |

fast CLI 在打开资源前拒绝非 pen、左单臂和协同模式，不能静默回退到右臂夹笔。双臂/观察 example 标记 offline_contract_only。本轮完成本地接口、账本、测试与协同审查，未恢复现场准入或实现通用物理执行器。

## 8. 证据与验收

发送、驱动接收、到位、物体结果分别记录。动作行视觉验证标为 pending_new_observation；下一轮模型判读标为 model_visual_report，不能改写为 vision_measurement。模型报告完成与独立任务成功分别记账；模型报告单独不会令通用 Recorder 的 task_success 为 true。历史回放没有动作后的物理后果。

在项目目录验证：

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_fast*.py'
python3 -m compileall -q src
```

根目录运行 `python3 tools/verify_bundle.py`。原始 source_manifest 哈希不变；本地修改和新增文件列入 optimization_manifest。本轮验证不调用真实模型、相机、ROS 或硬件。
