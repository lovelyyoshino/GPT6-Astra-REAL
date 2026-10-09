# GPT6-Astra-REAL

后续开发统一放在 **`/home/agilex/GPT6-Astra-REAL`**。这里整合了 `piper_right_pick_demo` 和 `piperx_cloth_demo` 的源码、任务经验与失败记录，并增加可复用的任务配方、持久化阶段账本、短经验提示和独立证据评估。目标是让后续复杂任务复用同一条软件流程。

**当前通用入口用于离线编排、回放和评估。** 通用任务、复杂组合及双臂观察保持 `offline_contract_only`、`execution_available=false`。既有单右臂抓笔 ROS 执行器与历史专项入口保留；新建任务 JSON 或通过回放，不会自动获得真机执行能力。本次整合没有启动机械臂、相机、ROS 或真实模型。

**用户要求实机任务时，现成能力直接复用，不重复验证。** Piper CAN、RealSense RGB 和已有控制宿主不需要每次重做源码审查、环境验证或 prepare；已有有效会话直接接续，初次接入只补必要状态，变化或异常只诊断相关项。不能把通用任务的离线标记扩大为整机不可在线控制。详见 [AGENTS.md 的在线控制使用核心](AGENTS.md#在线控制使用核心)。细微偏差依据执行器现有容差处理；异常立即停止新增指令，通过已有适用停止/保持机制读取回执，不将断连等同实机停止。

## 从这里开始

### 查找和使能 CAN

根目录脚本 [can_enable.sh](can_enable.sh) 默认依次完成 **旧任务软件状态归档重置、CAN 通信开启、所选机械臂关节电机使能、六关节实际使能反馈核对**。无参数运行时显示当前接口、状态和设备路径，按提示输入序号、接口名或 `all`。需要管理员权限时在系统提示中输入 sudo 密码，密码不会保存在脚本中。请在现场查看机械臂、物体和周围空间后运行；模式接管和电机使能可能引起姿态变化。

```bash
cd /home/agilex/GPT6-Astra-REAL
./can_enable.sh                   # 交互选择：CAN + 关节电机使能
./can_enable.sh --list            # 只查找和查看状态
./can_enable.sh can0              # 开启 CAN，并使能绑定到 can0 的机械臂
./can_enable.sh --all             # 开启 CAN，并使能两臂关节电机
./can_enable.sh --all --can-only  # 只开启 CAN 通信（原脚本行为）
./can_enable.sh --all --dry-run   # 仅预览命令
./can_enable.sh --all --reset-only # 只重置软件状态，不开启 CAN 或使能
./can_enable.sh --all --keep-state # 保留旧状态，走原有任务接续检查
```

2026-10-09 用户明确要求此启动脚本重置旧软件状态。默认将上次电脑启动前的 `runs/pair_sessions.sqlite` 完整备份到新建的 `runs/state_reset_*/pair_sessions.before.sqlite`，记录 SHA-256，再在同一数据库事务内清除旧任务、owner、软件故障、持物关系、接续和计数。旧日志、图片、配置及原始项目保留。选择单臂也会重置整对任务的软件关系，因此重置后的首次启动要求双臂均为待机、六关节及夹爪全部失能；只向所选臂发送。重置不发送硬件命令，不回零、不松爪，也不把旧夹持写成已经物理释放。

重置后先保留新的启动门禁；现场空载、无接触、断电重启及看护确认与新反馈都成立，才复用原执行器使能。`--reset-only` 完成后可再次普通运行脚本继续启动。正在运行的控制宿主、本次电脑启动后的任务活动、未决发送，以及本次重置后的失败/未决启动不会被重复运行抹掉；完整成功的首次启动记录保留。随后仅机械臂再次断电上电、电脑未重启时，默认运行也会转入下述独立断电启动流程，登记新事件并重新核对现场与反馈，不重放首次启动或重复清账本。`--list`、`--dry-run`、`--can-only` 不重置账本。下文保留历史的接续方式也可用 `--keep-state` 选择。

默认比特率为 `1000000` bit/s；其他比特率需搭配 `--can-only`。已开启且配置一致的接口直接跳过，但仍继续检查电机状态。左右臂来自 `projects/piperx_cloth_demo/configs/robot.json`，运行时核对 USB 绑定，不能仅按接口编号推断。现有启动入口需要双臂反馈，因此选择单臂时也会开启另一臂的 CAN 通信供只读监测，电机使能只发送给所选臂。

电机阶段使用现有 `pi0_infer` Python 3.10 和启动执行器，普通路径为 `robot_startup_arms` / `robot_startup_arm`，断电接续路径见下文；保留原有静止、反馈、低速、单次发送及持久回执约束。已 CAN 控制且六关节使能的臂不重复发送。部分启动、报警、反馈缺失、当前控制宿主占用会报出具体原因并停止，不自动重试、清故障、抢占、关闭接口或重命名。单次启动可能同时使能夹爪驱动，按实际回执报告，不发送夹爪开合目标。

**只给机械臂断电再上电、电脑不重启也已支持。** 运行 `./can_enable.sh --keep-state` 并选择接口或 `all`。存在同一次电脑启动内的旧任务记录时，脚本进入 `robot_startup_after_arm_power_cycle`：检查当前有效任务范围、旧失败的完整发送回执、活动控制进程、未决发送/保持与持物关系；随后按新反馈只选择待机且六关节和夹爪全部失能的臂。左臂失能、右臂已使能时只启动左臂，不要求右臂也断电。

终端会明确显示所选臂，请现场操作者确认“本次已对所选臂重新断电再上电，两臂无持物或接触负载，净空且有人看护”，输入 `yes` 后才调用原启动执行器。CAN 待机反馈不能单独证明发生过物理断电，因此保留这项现场确认；非交互调用不自动确认。每次断电事件有独立 ID，首帧发送前在原数据库追加 `pair_arm_power_cycles` 占位。同一 ID 不重发；成功后再次实际断电可用新的确认和失能基线启动。新事务失败、进程丢失或结果未决时禁止换 ID 重试。此入口只启动电机，原任务 owner、故障、目标、预算均保留，任务运动仍需适用的独立接续。

电脑重启后留下旧任务 owner/故障时，脚本会检查全部有效会话及其时间、当前控制进程、未决发送和持物记录。仅所有旧任务活动均早于本次电脑启动、无活动进程/未决发送/未解除持物时，可进入 `robot_startup_after_host_reboot`。实际执行前会请现场操作者输入 `yes`，确认机械臂也已断电重启、当前空载无接触、周围净空并有人看护；电脑启动时间不能代替这项确认。原启动执行器仍须从新反馈确认健康、静止、待机及完整失能，才会发送。

重启接续每臂每次电脑启动只允许一次有发送意图的使能事务，首次模式发送前持久占位；中断、部分发送、失败或结果未知都禁止本次启动内重发。原任务 owner、故障、目标和预算全部保留，原任务运动入口仍受旧故障约束。已使能臂仅查看并跳过，不消耗新的启动事务。

已有启动失败记录时，若 CAN 接口均已开启，脚本仍可通过只读反馈确认电机状态：所选臂均为 CAN 模式、六关节已使能且反馈健康，则显示“已使能，跳过发送”并正常退出。还需使能时按上述适用的断电接续分流；旧失败的部分/未知发送仍拒绝，故障不会因只读确认或新启动成功而清除。需要开启关闭的 CAN 接口时，仍先检查控制占用和启动记录。

脚本不发送关节位置、归零或停止命令，不启动相机、ROS 或模型。成功表示六关节使能位已收到并核对；不能将其写成位置锁定或保持能力已验证。`--dry-run` 仅预览配置与路由，不证明现场或实际使能已通过。

**使能不要求机械臂处于原点。** 偏移检查比较本次使能前后的位置，检查使能期间是否出现意外变化；它不比较零位，也不会把机械臂送回零位。若命令已发送、六关节使能位已收到，但后续偏移检查未通过，脚本分别显示末次回执中的使能数量和检查原因，完整数据保存在启动回执文件。此时仍按启动异常退出、不重发；报错不能理解为电机已失能或机械臂已停止。

### 任务入口

```bash
cd /home/agilex/GPT6-Astra-REAL
./astra catalog
./astra plan --recipe tasks/turn_faucet.json
./astra session --store artifacts/session.sqlite init --run-id demo --recipe tasks/pen_in_holder.json
./astra session --store artifacts/session.sqlite current --run-id demo
./astra replay --recipe tasks/sort_two_objects.json --out artifacts/replay_sort.json
./astra evaluate research/task_eval_placement_example.json
./astra experiences --task pen --phase INSERT
```

`plan` 展开任务合同；`session` 用同一 `store/run-id` 保存阶段、事件和总预算；`replay` 检查离线流程；`evaluate` 审核声明证据的来源与时序；`experiences` 查看有界历史建议。示例评估输入是合成数据，不能证明真实抓取成功。以 `./astra --help` 为参数依据。

| 配方 | 目标与复用点 |
| --- | --- |
| [pen_in_holder.json](tasks/pen_in_holder.json) | 抓笔放入笔筒，区分抓稳、插入、释放和稳定 |
| [can_on_lid.json](tasks/can_on_lid.json) | 空罐底部朝下立在杯盖上，目标支撑与原支撑分开 |
| [charger_in_unpowered_socket.json](tasks/charger_in_unpowered_socket.json) | 抓取充电头并插入无电插座；无电属于现场条件，不能从 RGB 推断 |
| [turn_faucet.json](tasks/turn_faucet.json) | 旋转关节物体至声明目标，避免误套螺纹进度判据 |
| [sort_two_objects.json](tasks/sort_two_objects.json) | 有限次重复抓放，演示多对象复杂组合 |

新增任务通过 recipe 描述目标、初态和有限步骤，组合已有 L2 操作。阶段可声明 `goal`、附加 `evidence`、`max_cycles` 和 `arm`；操作前提从注册表继承，不能由 recipe 改写。全局预算由 session 冻结，recipe 不提供全局 budget 字段，也不接受任意 Python、历史运动坐标或设备授权。新增物理技能仍需单独实现和验证适配器。

## 共用流程

```text
任务 recipe / 冻结目标与初态
  → 持久化 session：当前阶段、角色、剩余预算
  → 当前 L2 有界操作 + 必要 L1 契约
  → 当前 RGB / 本体状态 / 上次动作结果 / 短历史建议
  → 单步模型提案 → 具备当前资格的执行适配器 → 回执与新观察
  → 阶段证据 → 终态稳定评估 → 分别记录任务、回位和录像结果
```

执行适配器是架构接口；当前通用离线入口不会打开它。

| 能力 | 实现与状态 |
| --- | --- |
| 通用任务与复杂组合 | recipe 复用操作注册表；新任务合同保持离线 |
| 跨命令继续 | `fast_task_session.py` 冻结合同；revision/event-id 防重复，重启不清预算 |
| 阶段闭环 | `fast_policy.py` / `fast_live_loop.py` 为既有单右臂 pen runner；通用任务不静默降级到它 |
| 历史经验 | `fast_experience.py` 选择至多两条相关建议；卡片及原证据均校验哈希 |
| 结果评估 | `fast_task_evaluation.py` 分开模型判断、独立 RGB 审核和仿真 oracle |
| 失败恢复诊断 | `fast_task_recovery.py` 区分视觉问题、确认零发送、未知发送和停止证据；仅提出有限候选，不自动回退或重发 |
| 通用录像要求 | `fast_recording_contract.py` 绑定任务、必需视角和事件覆盖；元数据审计与视频完好、任务成功分开 |
| 多臂协作 | worker、peer、observer 角色合同；通用实机适配待验证 |

模型继续负责 RGB 视觉判断。经验不含可重放目标，不提供本轮授权、接触或成功事实。观察臂改善视野不清零工作臂的无进展计数。仿真的物体坐标、关节目标与奖励留在独立 grader，不能进入 RGB-only / calibration-free 控制输入。

## 两个旧项目的结果与教训

| 历史任务/问题 | 可核查结果 |
| --- | --- |
| 笔放入笔筒 | ROS 逐段执行成功，包含现场前后对齐指导；保留人工参与和容器变体。[结果](evidence/piper_pen_repeat_video_20261005/new_holder_result.json) |
| 空罐立在杯盖上 | 原记录确认松爪、撤离后独立稳定；杯盖存在由用户确认，等待与录像分段单独保留。[完成记录](evidence/local_integration_20261007/selected/can_task_completion.json) |
| 充电头插入无电插座 | 抓取、抬起已有证据，最终未形成插入成功；工具轴与持物轴分开判断。[结果](evidence/local_integration_20261007/selected/charger_final_outcome.json) |
| timeout / stop / hold | 早期超时处理后出现下落；另一次客户端退出后旧目标继续运动。退出、断连、失能不能当作已验证停止。[事故](evidence/piper_right_pick_demo/baselines/model_direct_vendor_ik/INCIDENT.md)、[后续审计](evidence/piper_right_pick_demo/runs/fast_repair_20261005_182331/command8_posthoc_audit.json) |
| 零发送拒绝与真正运动失败 | 保留原记录，不改写失败或自动重发不确定命令。[零发送复核](evidence/local_integration_20261007/selected/can_zero_tx_review.json) |

原项目 `/home/agilex/piper_right_pick_demo`、`/home/agilex/piperx_cloth_demo` 原样保留。原始视频、完整 `runs/`、逐帧图片和大体积 CAN 数据继续留在原处；新目录收录源码及精选证据，用[来源索引](evidence/local_integration_20261007/source_inventory.json)和[罐任务录像索引](evidence/local_integration_20261007/selected/can_recording_index.txt)关联原文件。此前放笔录像位于 `/home/agilex/piper_pen_repeat_video_20261005`。

## 参考研究

并行分析与交叉校对见 [通用化综合](research/generalization_synthesis.txt)、[任务评估](research/task_eval_review.txt)、[经验机制](research/memory_review.txt)、[组合架构](research/composition_review.txt)。对应 JSON 保存已核查版本、commit、源摘要和访问限制。

每项参考现在还对应一份[思想—代码—测试—待完成项清单](research/reference_adoption.json)，可用 `./astra references --source arx5` 查看。ARX5 的 [18 个任务/变体核查表](research/arx5_experiment_patterns_round2.json)区分操作要求与实验实际结果；擦板发生滑动不等于擦除，夹稳充电器不等于插入，用户允许残留也不意味着倒入量已测定。

按任务查看，例如 `./astra references --source arx5 --task drawer-push-pull`。这些研究材料按需查询，不会自动全部塞进每轮模型上下文。

| 原始资料 | 采用的原则 |
| --- | --- |
| [GPT6-ARX5](https://github.com/zijianzhang30/GPT6-ARX5) | 任务变体、角色、逐段验证、释放后稳定、回位与录像分别记录 |
| [ManiSkill](https://github.com/mani-skill/ManiSkill)、[TurnFaucet-v1 源码](https://github.com/mani-skill/ManiSkill/blob/main/mani_skill/envs/tasks/tabletop/turn_faucet.py) | 有限任务、清晰完成谓词、控制输入与仿真评估状态隔离 |
| [Zetta-Embodiment](https://github.com/air-embodied-brain/Zetta-Embodiment) | 诊断、候选改动、独立检验与接受改动分开 |
| [Self-Harness，2606.09498](https://arxiv.org/abs/2606.09498) | 从轨迹问题提出小改动，保留基线和回归；其研究对象是软件 agent |
| [PhysicalRSI](https://mmlab.hk/research/PhysicalRSI) | 持久技能与每轮状态分离、有界阶段；仅项目页内容获核查，版本与复现仍有限制 |
| [RoboICL](https://github.com/Mosi-AI/RoboICL) | 有界经验上下文，区别示范、当前观察与执行回执 |
| [Code as Policies](https://code-as-policies.github.io/) | 用明确 API 组合任务；本地采用受限 JSON 配方 |
| [Galbot 的 Astra 具身评估，2609.38537v1](https://arxiv.org/html/2609.38537v1) | 分别衡量发现错误、提出纠正和验证真实效果 |

这些原则用于改进宿主程序。当前没有引入物体三维定位、手眼标定、传统视觉检测或在线修改执行代码，也不以文献结果替代本机实验。

## 局部恢复与录像

```bash
./astra recovery-plan research/recovery_example_round2.json
./astra recording-plan --run-id task-001 --task can_on_lid --include-left
```

前者是合成输入的诊断示例；看不清或抓取失败时，候选是重新观察、重新对齐等当前局部操作。未知/部分发送、超时、未验证保持、故障锁存或预算耗尽会阻断继续提案。该接口不更改阶段、不消费持久预算、不执行实体回退；未来宿主须把新提案、计数和当前执行证据接入同一 session。

后者只输出录像合同：归零阶段不录，从初始姿态开始任务时录右腕第一视角与机身对面第三视角，左腕可列为额外要求；模型筛选视角不减少本地保存。实际三路 worker 已保留，但通用合同没有启动它，现有 worker 仍要求三路相机。`./astra recording-audit <JSON>` 审核时间戳、帧和事件的声明覆盖，不解码视频、不证明画面或任务成功。控制器退出不应直接结束证据采集；异常后的独立持续录像服务仍待接入。

## 开发与验证

```bash
cd /home/agilex/GPT6-Astra-REAL
python3 tools/verify_bundle.py
cd projects/piper_right_pick_demo
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_fast*.py'
python3 -m compileall -q src
```

本次整合验证见 [generalization_validation_20261007.json](metadata/generalization_validation_20261007.json)；旧 `optimization_validation.json` 保留历史含义。[source_manifest.json](metadata/source_manifest.json)保留原始来源哈希，本地改动由 [optimization_manifest.json](metadata/optimization_manifest.json)单列。离线测试不代表新任务真机成功、停止资格或性能提升。耗时、调用减少比例和成功率须用目标与现场条件一致的新实验测量，未知项保持未知。

历史全量测试尚未全部通过：部分测试依赖未迁入的旧 `runs/` 证据，旧 piperx 有 Python 3.8 与较新标准库 API 的兼容问题及工具 schema 差异，个别旧入口失败仍待定位。新增离线层和历史全量回归分开记录，不为得到通过结果删除检查或修改原项目。

在本目录进入 Codex，先读 [AGENTS.md](AGENTS.md)、[任务技能](.agents/skills/piper-task-pipeline/SKILL.md)和[原子技能索引](ATOMIC_SKILLS.md)：

```bash
codex -C /home/agilex/GPT6-Astra-REAL
```

`projects/` 保留两个项目结构；`vendor/` 保留厂家源码和许可证。真实 ROS、RealSense 与模型环境有各自依赖及路径绑定，详见 [OPTIMIZATION_GUIDE.md](OPTIMIZATION_GUIDE.md)。复制源码或安装 Python 包不会重建这些依赖及当前资格。
