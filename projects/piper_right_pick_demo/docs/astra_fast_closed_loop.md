# Astra fast closed-loop：审计、实现与验证

最新补充（2026-10-05 18:17）：用户重启设备后，右臂已通过独立的人工看护 ROS 单次回零路径到达六轴 0°；ROS 4秒及独立原始 CAN 约3秒确认，随后只读复查仍正常。当前驱动是 `ros_low_speed_entry`、速度 cap 1%、`gripper_exist=false`，右六轴已使能、夹爪未使能。这次成功只属于回零，**优化抓笔实验尚未开始**，不属于 `astra_fast_closed_loop` 的真实动作测试。

重新审查确认，当前 fast 闭环仍限于 MockRobot + HistoricalRGBSource；live CLI 与真实 dispatch 均被门禁阻止。除了运动中断保持、实际运动限制与执行期间独占资格，还存在实际接线缺口：当前回零驱动不符合 fast ROS 适配器的固定入口/遥测/夹爪合同；宿主需处理 observe/advance 并对齐动作完成回执；真实运行汇总不能使用写死 nonphysical 的报告；相机 worker 尚无与连续录像共享的采集循环。这些缺口不能通过删除门禁、调整配置或一次回零成功消除。此前“已完成”的表述只适用于离线控制器和真实感知—模型提案联调，不适用于端到端真机闭环。详见 [当前审查](../runs/fast_readiness_after_home_20261005/readiness.md)、[回零实测证据](../runs/ros_manual_home_20261005_180833/home_result.json)。

下文保留此前17:35审计记录；其中“尚未归零”与旧 ROS 会话状态仅代表当时状态。

当前状态（2026-10-05 17:35）：独立 `astra_fast_closed_loop` 已完成离线验证，并接通真实三路RGB、右臂ROS反馈和指定 `gpt-6-astra` 的官方Codex登录通道。两次实时图像决策通过schema/phase校验，分别耗时9.156秒、11.314秒；均为非运动提案，实机命令0条。ROS单动作发送/回执等待代码已实现，但仍受尚未资格化的保持和现场运动约束门禁保护，不能用配置开关解锁。尚未归零或重新抓取。原 `model.py` 和原模式保留；新后端为 `fast_model.py`（Responses）及 `fast_codex.py`（Codex CLI）。

## B：优化前到底做了什么

| 审计项 | 发现 |
| --- | --- |
| 完整闭环调用次数 | 当前 CLI 没有自动模型闭环。`decide` 每次一次请求、只输出提案；`attempt` 观察后 blocked。已有五份 Recorder 报告均 calls=0，真实对话中的 Astra 次数无法从机器人日志还原。 |
| 每轮文本 | 整个 instruction、整个 observation、动作合同、最多4条 history。CLI 实际未传 history。不是每次发送完整对话；重复主要来自任务说明和完整元数据。 |
| 每轮图像 | observation 中全部当前 RGB 原文件，逐个 base64；没有旧图、没有缩放。深度数组未附，但 metadata 含深度路径、内参及颜色检测结果。 |
| 连续上下文 | 未使用 `previous_response_id`；未配置 reasoning effort、严格 JSON schema。旧 max_calls 只在一个 Client 实例内累计，跨 CLI 启动重置。 |
| 等待来源 | HTTP 默认30秒上限，无重试。观察已并发读取机器人和相机，但每轮新建/关闭 Rig；RealSense 启动每路预热5帧，wait_for_frames 每次上限5秒。ROS取图上限8秒、轮询20ms；机器人轮询10ms；ROS master探测上限2秒。上限不能当实测耗时相加。 |
| 原有计时 | 模型计时在图像编码后才开始，包括请求、网络等待与解析，不是纯推理。缺少采图、编码、逐步执行/等待、phase耗时。 |
| 观测期限 | 当前旧配置0.8秒。长模型等待后不能继续把原图当新鲜执行依据，也不能仅为测速放宽期限。 |

### 159 mm 下落的触发链

`baselines/model_direct_vendor_ik/INCIDENT.md` 和原反馈记录表明：第4段163 mm MOVE_L运行40秒，只前进约18 mm；超时前Z=251.811 mm。旧 `abort()` 先尝试0.45秒静止确认，失败后发送 `MotionCtrl_1(1,0,0)` 快速急停，控制模式转待机，最低Z=92.761 mm，下降159.050 mm，stop.confirmed=false。计划中的下降抓取和闭爪尚未发送。

当前工作脚本曾仍含这个危险分支，暂停文件只挡main，不挡直接调用Executor。新工作版本现已删除自动快速急停：异常封锁后续TX，只做被动记录，明确 `hold_unverified=true`、`target_cancelled=false`、`onsite_intervention_required=true`。main、Executor构造、step和发送入口均封闭真实执行；删除暂停文件也不会解锁。基线存档保持原样。

这项修复**不等于实现了保持**。停止发送或关闭通信不能证明固件已接收目标被取消，单元测试也不能证明重力作用下保持。当前不能开始自主连续真机，也不能先归零来跳过这个前提。

## 新模式

阶段严格采用：INIT → APPROACH_PEN → ALIGN_PEN → PREGRASP → GRASP → VERIFY_GRASP → LIFT → APPROACH_HOLDER → ALIGN_HOLDER → INSERT → RELEASE → VERIFY_SUCCESS → DONE；另有 RECOVERY。模型只得到当前局部目标，不重复完整规划。

每轮输入是白名单 `controller_state`：phase、机器人实际状态、夹爪状态、上一动作、简短结果、retry_count、最多240字memory，以及执行层提供的数值动作上限。后者只描述允许位移/旋转/速度/夹爪范围，不是物体位置。整段历史、推理、内外参、深度、检测结果、历史目标坐标均不透传。

模型使用 Responses 的 `text.format` strict schema，每次一个结构化动作。普通响应四个字段：phase/action/arguments/confidence。异常状态另要求最多240字 explanation，仍不索取思维过程。`phase`指当前阶段；同阶段安全运动可携相邻 next_phase，视觉验证边不能跳过。执行完成才提交切换；闭爪和松爪仅机械进入验证阶段，不自动宣称抓住或成功。

每步都重新采图，覆盖靠近笔、抓前、闭爪后、抬起后、靠近笔筒、插入前后、松爪后和退开后的检查点。抓取、插入、松爪等关键阶段不能由一个chunk包办。VERIFY_SUCCESS→DONE还要求本轮已松爪、已退开，并由模型看新图给出success。

动作包括 observe、advance、move_eef、move_eef_chunk、gripper、pause。EEF姿态是右基座下的 **driver end reference**（米/弧度），不冒充已标定夹爪尖TCP。chunk最多3点，仅限两个APPROACH阶段，必须是模型认为可见净空且无接触的局部路径；真实路径安全资格尚未建立，因此当前只在mock中执行。

阶段位移上限取已配置硬限值的比例：APPROACH 1.0、ALIGN 0.5、PREGRASP/VERIFY_GRASP/RECOVERY 0.15、INSERT 0.1、LIFT/退开验证 0.5。chunk累计路程不能超过阶段单次上限，不因分段而扩大许可。工作区、关节反馈范围、速度、姿态变化、夹爪范围/力度、新鲜反馈和一次性执行票据仍在独立安全层。mock限值只是软件测试夹具，绝不自动成为实机限值。

相机在会话内复用；三路继续采集、保存原分辨率RGB。模型按阶段选择视角：精细抓取/验证通常front+right_hand，接近笔筒以front为主；异常升级三路。RGB-only模式不运行颜色检测、不启用深度；ROS RGB模式也不依赖CameraInfo/内参。未做裁剪、缩放或隐藏传统视觉定位。

低风险phase采用low或medium；GRASP、INSERT、VERIFY及RELEASE采用high；异常high、连续重试xhigh。模型明确固定 `gpt-6-astra`，不静默回退。`store=false`，每次短状态包，不传 `previous_response_id` 或加密reasoning历史。官方支持逐请求 `reasoning.effort` 和 strict schema；串接previous_response_id仍会累积上下文、历史输入仍计费，因此本控制器不采用。[Reasoning](https://developers.openai.com/api/docs/guides/reasoning)、[Structured Outputs](https://developers.openai.com/api/docs/guides/structured-outputs)、[Conversation state](https://developers.openai.com/api/docs/guides/conversation-state)

## 计时与退出

每步 `steps.jsonl` 保存用户要求的step_id、phase、timestamp、模型请求/响应时间、agent_decide_s、image_capture_s、image_encode_s、robot_execute_s、robot_wait_s、total_step_s、输入/输出/reasoning token、选用相机、动作与参数、previous_result、confidence、phase transition、retry_count。模型、编码、采图和执行/等待分开计时，未知tokens为null。

`fast_summary.json` 汇总总时间、模型请求时间、机器人执行与等待时间、相机时间、其他时间、调用数、平均/中位/最大模型延迟、动作数、阶段耗时、recovery数及task_success。模型时间是接口墙钟时间，无法从API用量拆出纯内部推理秒数。mock报告 `task_success=null`；未尝试的真机也不是失败样本。

总step、模型请求、phase重复、retry、recovery和墙钟均有预算；迟到响应不执行，异常/部分执行锁存后不自动重发，pause不因置信度低而被忽略。1800秒是停止预算，不是已经证明成功任务小于30分钟；HTTP timeout也不是每轮必然等待。

## 可复现命令

从项目根目录执行。以下默认只用保存图片、脚本决策和mock机器人，无模型费用、无硬件命令：

```bash
PYTHONPATH=src python3 -m right_pick.cli --runs runs/astra_fast_closed_loop astra_fast_closed_loop --execution replay --observation /home/agilex/piperx_cloth_demo/runs/obs_16122eb642664e9bae78e2d92d0033ce/observation.json
PYTHONPATH=src python3 -m unittest discover -s tests
PYTHONPATH=src python3 -m right_pick.cli --runs runs/astra_fast_closed_loop astra_fast_closed_loop --execution prepare
```

prepare/live当前都记录阻塞并返回2，且不打开CAN/ROS/相机或使机械臂归零。不存在SDK回退。

`configs/astra_fast_closed_loop.example.json`是单独配置，未修改旧site.local。显式 `--model responses` 会对历史图像和mock状态发真实付费请求，需要环境变量 `OPENAI_API_KEY`；它仍是非物理回放，不能据此计算抓取成功率。当前环境未配置该凭据，本次没有真实API调用。

上段描述最初离线验证。后续接入的 `--model codex` 使用本机已登录的官方Codex CLI，无需从登录文件提取凭据。固定版本0.160.0，精确请求 `gpt-6-astra`，不提供模型回退。每轮新建空目录中的ephemeral会话，禁用工具/项目文档/外部连接，使用严格输出schema；限制工具目录只修改工具能力，不修改模型ID或引入物体信息。CLI提供逐次reasoning effort；内部固定提示、启动及内部图像编码仍计入决策时间。CLI事件未回报actual_model时保持null。CLI无本实现可用的max_output_tokens参数，使用短schema、响应字节上限和timeout，不声称强制了输出token上限。[官方非交互使用说明](https://developers.openai.com/codex/noninteractive)、[官方登录说明](https://developers.openai.com/codex/auth)

真实感知—模型接口测试（不会调用execute，不推进实际phase）：

```bash
source /opt/ros/noetic/setup.bash
source /home/agilex/piper_gpt/devel/setup.bash
PYTHONPATH="$PWD/src:$PYTHONPATH" /usr/bin/python3 -m right_pick.cli \
  --config configs/astra_fast_live.local.json --runs runs/astra_fast_live \
  astra_fast_closed_loop --execution live-check --model codex \
  --fast-config configs/astra_fast_codex.local.json --phase INIT
```

ROS运行使用系统Python；RealSense用独立camera-only worker运行于已安装pyrealsense2的Python3.10。会话中复用三路流，关闭时只清理自身相机进程。`PYTHONPATH`必须保留ROS环境，不能覆盖成单独的src。模型返回后重新取机械臂反馈供本地命令编码校验，**不**把它当作RGB刷新。完整raw CAN反馈及命令日志只留本地，不进入模型包。测试报告分别记录schema、phase、命令编码、数值边界和物理许可，非运动合法提案通过不等于运动链通过。

| 当前真实接口测试 | INIT | APPROACH_PEN |
| --- | --- | --- |
| 单轮总时长 | 12.285秒 | 14.306秒 |
| 模型接口时长（含CLI开销） | 9.156秒 | 11.314秒 |
| RGB采集及本地输入准备 | 1.759秒 | 1.766秒 |
| 模型调用 | 1 | 1 |
| 输入 / 输出token | 8474 / 43 | 8490 / 37 |
| 发送视角 | front + right_hand | front + right_hand |
| 提案 / 置信度 | advance→APPROACH_PEN / 0.96 | observe，evidence=unknown / 0.90 |
| 派发实机命令 | 0 | 0 |

这两轮平均决策10.235秒，不是完整任务耗时或速度改善对照。APPROACH_PEN请求继续观察，未提出运动；不能据此声称已经测过运动或成功率。证据：[INIT](../runs/astra_fast_live/20261005T093351Z_8132bad686a8/fast_summary.json)、[APPROACH_PEN](../runs/astra_fast_live/20261005T093429Z_f05c9f6683bf/fast_summary.json)。三路原图、紧凑输入包、提案及反馈均在各run内保存。

回放保存原照片和source_observation；重复使用照片明确标记historical/source_reused。脚本走通只检查phase/schema/计时/有限退出，照片不是mock运动后的模拟画面，不验证Astra视觉能力。

## 对比与后续实机前提

最终验证：原198项测试通过；加入新模式和超时分支测试后，265项全量通过。保存的三视角RGB完成14个脚本决策、10个mock运动/夹爪动作，模型API调用0次、实机命令0条、task_success=null。附件选择为26张，对照同样14轮每轮3张的42张减少38.1%；这只是图像附件数量，不是模型调用降幅或延迟改善。原有414份证据校验值全部未变。[验证记录](astra_fast_validation.json)、[回放步骤](../runs/astra_fast_closed_loop/20261005T090129Z_b6018b229c70/steps.jsonl)、[完整测试日志](../runs/astra_fast_closed_loop/20261005T090129Z_b6018b229c70/all_tests.log)、[JSON对比](../runs/astra_fast_closed_loop/20261005T090129Z_b6018b229c70/comparison.json)。

| 指标 | Baseline：上一轮有现场指导的ROS放笔 | Optimized：当前状态 |
| --- | --- | --- |
| 完整任务时间 | 无统一任务时钟；录像进程窗口3534.98秒，约58分55秒 | 尚未真机测量 |
| Astra调用次数 | 未记录；49条ROS任务命令不能替代 | 尚未真机测量 |
| 模型推理时间 | 未记录 | 尚未测量；新日志将给出模型接口耗时 |
| 机械臂运动时间 | 无纯运动计时；48/49条命令“意图→稳定反馈”合计47.160秒 | 尚未真机测量 |
| 结果/成功率 | 该轮成功，含现场前后位置指导；不能估计自主成功率 | 软件回放及真实感知—模型联调通过，尚无实机成功率 |

`scripts/compare_fast_runs.py`生成可追溯JSON对比，缺失指标保持null，不把mock耗时、脚本决策数或剪辑视频时间用于KPI。<30min、调用下降≥30%、平均决策更快、保持成功，均尚未得到实机证据。

当前右臂/三相机只读绑定已核验，真实Astra后端已接通；ROS执行适配器含单次发送、精确驱动命令序列/帧回执校验、每步新鲜反馈、失败锁存，已用FakeROS测试。P移动保留最后明确的夹爪目标，避免把夹住物体后的实测开口重新作为夹爪目标。真实chunk执行尚未实现，提案编码会拒绝，不会悄悄拆成整段运动。

开始右臂归零和新真机轮次前仍缺：适用当前ROS控制器的可验证中断/保持机制（包含timeout、进程退出/失联、已接受目标的处理），真实工具路径净空与独立运动限值，以及实际派发期间的控制独占验证。当前ROS路径已核对不包含旧SDK超时自动急停分支，但其失败封锁只阻止后续TX，不能取消固件已接收目标，也不能证明中断保持。原hold记录qualified=false，不能用布尔配置或mock结果代替。归零轨迹也须通过相同安全层。此处约束只限定机器人执行，不提供物体坐标或相机标定信息。

原始代码快照位于 `/home/agilex/piper_right_pick_snapshots/pre_fast_20261005_164527`；其中 `preserved_evidence_sha256.json`记录旧runs/baselines的414个文件。失败的离线开发回放也保留，未覆盖历史实验。

接入前再次快照：`/home/agilex/piper_right_pick_snapshots/pre_live_integration_20261005_171841`。开发时两次固定JSON烟测在CLI启动警告被拒绝，第三次成功（10.636秒）；另一次live-check因启动命令覆盖ROS的PYTHONPATH而在模型请求前失败。失败记录均保留，不算物理尝试。系统Python的老SciPy导致三个旧轨迹测试类初始化失败；全量测试使用项目原有aloha Python环境，真实ROS链路使用系统Python并单独验证。

最终接入回归：aloha Python下338项全量通过（26.273秒）；ROS系统Python下132项fast专项通过（0.239秒）。新增73项覆盖单动作ROS收发与失败锁、官方CLI严格响应与隔离、实时反馈/图像时序、提案编码失败、相机子进程生命周期及清理错误。接入前662份runs/baseline文件SHA256全部未变。[接入验证JSON](../runs/astra_fast_live/validation_20261005_173931/validation.json)、[全量测试日志](../runs/astra_fast_live/validation_20261005_173931/all_tests.log)、[对比JSON](../runs/astra_fast_live/validation_20261005_173931/comparison.json)。
