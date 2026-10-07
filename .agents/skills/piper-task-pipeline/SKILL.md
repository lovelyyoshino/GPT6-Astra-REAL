---
name: piper-task-pipeline
description: 在 Piper 交接包中通过 Codex 规划、执行或复盘桌面任务，按任务 pipeline 调用当前有界操作；覆盖单臂、双臂和主臂执行辅臂观察的能力分流与预算退出。
---

# L3 任务入口

从项目根目录的 `AGENTS.md` 获取本项目的 Codex 使用方式和实机边界。固定本轮 task、mode、角色、初态、目标及总预算；已有明确指令直接采用，不重复采访。

本项目长期采用当前图像准入、执行一段、用新图和反馈接续的监督方式。每轮只提出当前有限目标，不先生成、采集或重放完整任务轨迹。不要把精确物体坐标、全场景米制模型或预先实测成功的轨迹当作所有阶段的统一前提；使用本阶段明确支持视觉监督的执行分支，保留设备数值与故障守卫。协议所需的当前发送状态仍须核对，未接通的分支如实报告。补齐适配时也按此方式设计；执行后的新图、本体反馈和回执用于本步复核及后续经验。

用户要求操作实物或在线控制 Piper 时，按 [在线控制使用核心](../../../AGENTS.md#在线控制使用核心) 直接复用现成 CAN、RealSense RGB 与唯一控制宿主。已有有效会话不重做能力验证，不把全量源码审查、环境重验或反复 prepare 放在每次任务前；初次接入只补缺失的必要状态，具体变化/异常只诊断相关项。每步照常获取当前 RGB 和反馈。通用离线标记不代表设备或所有历史适配器不可用，以下离线命令也不替代实机执行。

任务起始需要处理无效控制进程，或新反馈显示待机、未使能、控制模式不匹配、缺首次目标或初始化后边界尾差时，读取 [在线接续分流](references/online-readiness.md)。复用现成 startup、模式接管、空爪准备及目标接续，只补缺少的步骤后继续；不将未使能直接报告为能力缺失。已知有效会话不重复走此分流。

用户要求任务录像时，按 [录像生命周期](references/online-readiness.md#录像生命周期) 在实际操作前复用三路 RGB 连续录像，任务正常或故障退出时均封口并核对真实覆盖范围。

按当前图像与操作阶段选择步长：用户期望远处厘米级及几度，近处约 1 厘米或更小，接触继续缩小。双爪空载且远离物体时，同一 RGB joint 入口显式选择 `motion_profile="coarse_approach"` 并提供当前 `far_from_target_observation`，单轴目标最多 3°、模型末端目标最多 2 厘米；它是独立的软件动作范围，不是默认小步档的别名。临近目标、持物或接触时不用此档。数值上限不保证任意姿态或方向都能走满，也不等于物体位移；每段后新图再决定，不预排多段冒充一个大步。具体调用、过程边界和验证状态见 [图像监督步长选择](references/online-readiness.md#图像监督步长选择)。

每段先简述计划幅度，回执后对照实测幅度并说明下一步修正依据；关节角、机器人法兰反馈和图像中的物体进展分开报告。按[逐段动作报告与经验更新](references/online-readiness.md#逐段动作报告与经验更新)保存来源，持续改进下一段判断及技能说明，执行中不改控制代码、阈值或预算。

区分运动暂态与最终到位。小偏差依执行分支明确支持的幅度、累计持续时间与独立反馈判断，允许范围内的短暂偏差记录后继续观察收敛，不立即判失败、不新增逐位一致要求、不反复确认或重发。图像监督首次初始化、普通接近/对齐及已接通的带载 joint 分支均有完整发送后的有界收敛观察，具体范围见 [在线接续分流](references/online-readiness.md)；不能把这项策略泛化到所有动作或改写旧失败。持续偏离、硬边界、反馈及接触异常仍停止新增运动指令，通过已有适用停止/保持机制获取回执；停止状态未知时明确报告，不能承诺退出或断连就已停住。

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

当前通用 pipeline 中，单右臂 `pen` 已有 phase runner 与 Codex 后端；其他 recipe 及协同模式标为 `offline_contract_only`。另有独立观测工具和历史专项在线入口，应核查当前宿主是否覆盖本次任务，不能仅据通用 pipeline 的标记退出。不能直接把离线账本接上硬件，也不能把旧专项资格套到新任务。

双任务臂的持久有界发送宿主已提供 `robot_pair_*` 固定工具，具体按 [协同分支](../piper-arm-coordination/SKILL.md) 接续同一 owner/run；它与通用 recipe 的离线标记分别报告。接触任务按 [L2 接触与微调](../piper-manipulation/references/bounded-contact.md) 直接推进已有适用入口：低速小步接触、看实际响应、有限微调，证据在动作后建立，不以“尚未接触成功”提前结束。当前已实现有界夹爪观测、按臂静态保持，以及受支撑开爪/继续开爪、分离确认和空爪撤离。probe 带物体身份后，新 RGB 语义与适配器新 trace 可经 `robot_pair_retain_grasp` 零 TX 保留原目标，另一空臂继续准备。开爪到位只记 `release_opened`；`robot_pair_confirm_release` 需要最后开爪后的新图与新反馈，之后每段 joint 撤离仍需当前空爪描述。已接通右臂图像监督拔出、搬移和插入的专用分支，左臂保持桌面插排；每段后必须用新图确认物体响应，不能从到位自动推进。该分支及带载后的受支撑释放见 L2 接触文档。`tasks/plug_transfer_left.json` 冻结本任务左右角色与物体证据顺序。

用户要求执行时，已有入口及现场资格覆盖本次任务就使用同一有效控制宿主直接推进；模型入口显式 `--model codex` 和 Codex 配置，沿用现有登录。确实缺执行适配或现场必要条件时，报告具体缺口及已有证据并结束该执行请求；用户要求修复缺口时进入对应有限诊断流程。不要把缺一个 Python 包、旧文档状态或“没有重新验证”当作整机不可用的结论。

结束分别报告任务、回位、发送/模型调用、耗时、人工参与与 `termination_reason`。`offline_contract_completed` 只说明账本通过，不能写成实机成功。详见 [层次和任务映射](../../../projects/piper_right_pick_demo/docs/ATOMIC_SKILLS_AND_DUAL_COORDINATION.md)。

针对当前 PiPER X 插拔任务，按 [实现设计](../../../projects/piperx_cloth_demo/docs/PIPER_X_JOINT_PATH_DESIGN.md) 区分代码实现、生产接通和实物结果。同连接空爪准备、限位查询、来源读取及 `robot_pair_initialize_joint_target` 首次目标建立已接入持久工具。首次初始化独立处理合法非零起点或适用的 J2/J3 启动恢复，普通关节路径的已知缓存及 hold 前提不用于阻断这个分支。初始化后的边界向内接续也已接通：宿主自动绑定当前初始化来源与真实目标缓存，沿用 `robot_pair_submit_once(kind="joint", operation="approach"/"align")`，不增加调用参数、不重复初始化消除尾差。具体范围及保持限制见 [在线接续分流](references/online-readiness.md)。

首次目标现在也支持显式 `admission_mode="rgb_supervised"` 的监督分支，按[在线接续分流](references/online-readiness.md)记录当前空载与本段整臂通道语义。它保留原关节、低速、相对位移和反馈限制，不要求先造齐米制现场文件；不会自动成为普通运动或带载资格。

官方参数从版本化固定文件复用，源插孔/左目标孔及每段接触响应从新 RGB 判断；不要求用户预先给出全部物体坐标，也不把未知现场尺寸填为默认值。普通空载 `joint` 接近/对齐已接入显式图像监督分支：同一 `robot_pair_submit_once` 使用 `admission_mode="rgb_supervised"`、当前工作臂的 `unloaded_observation` 与整臂通道 `corridor_observation`；按新图给本次有限目标，无需先提供或重放实测轨迹。真实缓存、设备限位、步长、反馈和期限仍由宿主核对，米制分支保持独立且不自动回退。图像监督带载段与新图响应已接通，最终插孔身份、双爪离开和两次稳定图仍需本任务的真实证据。受支撑释放的 `released` 只解决夹持关系，不证明插对目标或最终稳定；旧 v1 机械释放不自动继承新语义。官方 SDK/FakeCAN 贯穿测试说明软件接通，不是实机资格；初始化、向内到位、静态持夹和释放均不等于拔插成功。

旧运行因已知完整初始化暂态失败且到期时，适用的人工确认接续见 [审计接续](references/audited-restart.md)。不得自动消费新预算；旧故障、回执、原期限和累计步数全部保留。

正常干净结束且原窗口已到期时，用户明确授权的新一轮可用 [pair_round 管理入口](references/audited-restart.md#干净结束后的显式新轮)追加新预算并保留原历史。用户明确要求“修复后重新计时”时，使用 `after_repair_before_online_execution`，在离线修复与检验完成后、首次在线执行前冻结起点；开始后重连或再次修复都不自动续时。已有明确的本轮次数、时限和计时方式授权直接采用，不重复询问，也不把新轮管理激活当成动作或任务成功。

图像余量不足的 `refresh_required` 只需刷新本段图像后重新决策，不重复设备准备。已完整发送的空载 RGB 动作仅因图像到期失败时，适用的原预算内接续见 [图像超时接续](references/audited-restart.md#原预算内的图像超时接续)；保留旧失败，不直接重发目标。
