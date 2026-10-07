# 单右臂夹笔工程：优化交接指南

本包用于源码审阅与后续优化。最新 fast 已有真实 ROS 闭环代码，也发生过一次真实任务运动；**连续实机运行准入现已撤回，优化版尚未完成抓笔放筒**。当前状态以[撤回记录](evidence/piper_right_pick_demo/runs/fast_repair_20261005_182331/continuous_admission_withdrawn.json)和[撤回后的现场配置](evidence/piper_right_pick_demo/runs/fast_repair_20261005_182331/fast_live_commissioned.json)为准，后者的 `physical_qualification.evidence_file` 为 `null`。旧 fast 文档中“尚无 live 循环”“只完成感知联调”等描述属于此前阶段。

当前源码包含本地优化，变更由 `metadata/optimization_manifest.json` 追踪，原来源哈希不变。已运行离线 fast 测试与契约验证，未调用真实模型或设备。包内保留小型结果、配置和审计摘要；指向未打包视频及原始反馈的路径仍是历史引用。

本轮已完成的修改：

- 默认使用现有 Codex 登录与 CLI；后端/配置协议不一致时在打开资源前拒绝。
- 新增 L3 任务、L2 操作、L1 原子、L0 适配四层结构，以及 18 个 ARX5 recipe、20 个组合和 25 个原子契约。见[完整映射及实现状态](projects/piper_right_pick_demo/docs/ATOMIC_SKILLS_AND_DUAL_COORDINATION.md)。
- 单右臂循环限制调用、阶段、观察、恢复和时间预算，模型输入只带当前阶段与短状态；派发后等待新图，不在到位行提前记视觉完成。
- 修正恢复预算的边界错误：原先设置允许 1 次恢复，却在刚进入 RECOVERY 时退出；现在允许该次有界纠错，第二个恢复阶段才停止。异常发送仍立即锁存，不自动重发。
- 新观测核对帧身份/新鲜度；模型视觉报告按真实来源记录。双任务臂使用 `peer_arm`，纯观察侧使用 `observer_arm`，禁止角色混用或静默降级。
- 四个分层入口和 25 个同名原子 Codex skill 已放在隐藏的 `.agents/skills`；见[可见索引](ATOMIC_SKILLS.md)。首次读完整单任务，之后按当前阶段逐层查询，避免每轮加载全部任务手册。
- `fast_task_session.py` 提供持久化的 init/current/contract/record/observer/end 命令；冻结任务与总预算，通过 revision 防止旧阶段提交，event-id 防止重复计数。跨进程恢复只重建离线账本，不续发任何物理指令。

以下是仍影响实机恢复的证据与缺口。通用 recipe、双臂和辅助观察目前是离线契约，不能因测试通过就记成实机支持。

先按下面顺序读代码。表内入口均在 `projects/piper_right_pick_demo/src/right_pick/`，底层驱动另列。

| 优先阅读 | 文件 | 关注内容 |
| --- | --- | --- |
| 总入口 | [cli.py](projects/piper_right_pick_demo/src/right_pick/cli.py)、[fast_cli.py](projects/piper_right_pick_demo/src/right_pick/fast_cli.py) | `astra_fast_closed_loop` 的 `replay / prepare / live-check / live` 分流；`--execution` 必填，live 必须指定真实模型。 |
| 真实闭环 | [fast_live_loop.py](projects/piper_right_pick_demo/src/right_pick/fast_live_loop.py) | 准入检查、新三图、新 ROS 状态、单次模型决策、重新核对状态、执行及回执、阶段推进、失败收尾。 |
| 动作和阶段 | [fast_policy.py](projects/piper_right_pick_demo/src/right_pick/fast_policy.py) | 接近笔、对齐、抓取、验证、搬运、插入、释放及最终验证；schema、阶段限制、视角选择和 reasoning effort。 |
| 真实执行 | [fast_ros.py](projects/piper_right_pick_demo/src/right_pick/fast_ros.py)、[fast_safety.py](projects/piper_right_pick_demo/src/right_pick/fast_safety.py)、[fast_qualification.py](projects/piper_right_pick_demo/src/right_pick/fast_qualification.py) | 固定右臂 ROS 路由、来源／会话校验、数值限制、单次发送、回执核对、资格证据。当前 live 仅支持单个末端目标或夹爪动作，禁用 chunk。 |
| 模型与图像 | [fast_codex.py](projects/piper_right_pick_demo/src/right_pick/fast_codex.py)、[fast_model.py](projects/piper_right_pick_demo/src/right_pick/fast_model.py)、[fast_observation.py](projects/piper_right_pick_demo/src/right_pick/fast_observation.py)、[fast_camera_worker.py](projects/piper_right_pick_demo/src/right_pick/fast_camera_worker.py) | Codex／Responses 两种后端、紧凑输入、三路 RGB 长驻采集；快照与连续录像共用流。模型只接收选定 RGB、状态及局部目标，不接收深度定位或手眼标定结果。 |
| 统计 | [fast_recording.py](projects/piper_right_pick_demo/src/right_pick/fast_recording.py) | 模型接口、采图、执行、等待耗时；发送尝试、确认回执、模型报告成功与任务证据的区别。 |

1. **P0：先补足全臂路径约束，再讨论恢复连续执行。**

   [command 8 审计](evidence/piper_right_pick_demo/runs/fast_repair_20261005_182331/command8_posthoc_audit.json)确认：只发送一次任务 ROS 命令，驱动在约 24.782 秒后确认到位，但客户端在发送后的等待阶段触发限制并退出。端点位移约 28 mm，J1／J4／J6 分别变化约 +47.1°／+69.4°／−68.1°。因此“末端小步”不能代表整臂小幅运动，也不能用最终到位抹去途中监测失败。

   优先离线审查 `fast_ros.py` 的目标限制与执行中监测语义，分别定义目标步长、关节变化、连杆／附件路径包络和异常结果。结合厂家 FK 比较连续关节解及路径候选，保留原失败作为回归案例。[离线几何复核](evidence/piper_right_pick_demo/runs/fast_repair_20261005_182331/command8_offline_geometry_review.json)支持近奇异位形放大这一解释，但仅分析前后端点，不能证明固件切换了 IK 分支，也不能还原实际路径或证明净空。不能通过增大阈值、填回证据路径或替换 hash 宣称问题已经解决。

2. **P0：分开处理“客户端停止”和“机器人已停止”，补齐故障后的观测。**

   该次客户端报告为 `action_count=0`、`control_commands_sent=null`、`target_uncertain=true`；事后驱动回执却证实一次真实发送和到位。两者描述的是不同确认阶段，应同时保留。相机随客户端结束，约最后 23.4 秒实际运动未录到，完整实际轨迹也未保留。

   优先检查 `fast_live_loop.py` 的收尾、`fast_ros.py` 的失败锁及 `fast_camera_worker.py` 的生命周期：失败后禁止后续目标，同时让独立被动观测有明确的终止条件，补齐动作结果对账。统计应区分发送尝试、驱动接收、最终到位和抓取成功。当前资格设计采用保留驱动／使能、让已接受的有限目标完成后不再发下一目标；这不等于即时保持或通用急停验证。证据见[原客户端摘要](evidence/piper_right_pick_demo/runs/astra_fast_physical/20261005T111800Z_4c757c312206/fast_summary.json)与上述 command 8 审计。

3. **P1：减少无进展的模型往返，先解决可见性和动作合同。**

   [实验对照摘要](evidence/piper_right_pick_demo/runs/fast_repair_20261005_182331/experiment_comparison_after_visibility_review.json)中，一轮 101.141 秒、11 次模型调用，模型接口累计 97.937 秒，实际动作数为 0；另有多轮因遮挡判断而 pause。这里模型接口等待是主要耗时，不能归因于机械臂速度。后续现场还纠正过前视图中“机器人右侧对应图像左侧”的理解，修改后的实验也不能标为全自主。

   当前默认第一次 `observe(unknown)` 后提示推进或说明具体阻碍，第二次终止，总计最多 24 次模型调用和 900 秒；常规输入只包含当前阶段。现场采图及状态读取后会再次核对总时限，避免预算已耗尽仍调用模型。模型返回后刷新机器人反馈并检查漂移，尚未重新让模型看场景；对等待期间人、笔或笔筒移动仍没有独立场景变化检测，不能给旧 RGB 重贴新时间戳。该假设和 RGB 年龄已记入运行日志。

4. **P1：测量模型后端开销，再决定怎样提速。**

   用户使用 Codex 访问项目控制，沿用已有登录，不需要 API key。Codex 路径每轮启动独立、禁用工具的 ephemeral 会话；固定模型 `gpt-6-astra`，当前适配器检查 `codex-cli 0.160.0`。`agent_decide_s` 包含 CLI、图像处理、网络及服务端等待，不是纯推理时间。后续在相同输入、schema 与推理等级下测量图像选择和会话开销；任何复用都须保留工具隔离、上下文边界和超时后不重放动作的合同。不要将更快的单轮决策直接写成更快的完整抓取。

5. **P2：统一评估口径，保留人工参与和未知值。**

   [历史放笔结果](evidence/piper_pen_repeat_video_20261005/new_holder_result.json)记录了松爪、退开后笔仍在筒中的成功证据，也明确记录现场前后对齐指导；它是监督下成功，不是自主成功率。对照摘要给出的重建视觉窗口约 2403.699 秒；3534.980 秒是录像进程窗口，包含准备与等待。49 条 ROS 命令不是模型调用数，48／49 条命令累计 47.160 秒是“意图到稳定反馈”，也不是纯运动时间。

   后续评估固定同一任务起止点，分别报告模型调用、接口等待、发送至稳定反馈、无进展观察、人工介入和最终视觉验证。基线模型次数及耗时缺失，因此目前无法计算调用下降比例或模型延迟改善；优化版没有完成任务，也尚不能证明小于 30 分钟完成或成功率提升。

6. **P2：整理环境与路径，但保持现场资格和源码分离。**

   两套项目的 local 配置已随包，`rg` 默认会受原 `.gitignore` 影响而隐藏这些文件；核查时使用 `rg --files --no-ignore`。最新撤回配置位于上文 evidence 路径，文件名中的 `commissioned` 不表示现在仍准入。原 `site.example.json` 仍有历史 can2，当前 fast 固定的是 can1／USB `1-6.3:1.0`，不能混用。

   迁移需明确替换原主机的相机 Python、Codex 二进制、图片路径及日志路径。`fast_ros.py` 还硬编码 `/home/agilex/piperx_cloth_demo/.../ros_resume_entry.py` 和 `/home/agilex/piper_gpt/.../piper_ctrl_single_node.py`，固定 SHA256 且拒绝配置覆盖；安全层依赖原始资格 collector、当前 boot ID、驱动 PID及会话 command log。证据摘要不能替代这些运行依赖，也不能直接迁移原主机资格。

   底层参考源码已补齐：[ROS 执行适配器](projects/piperx_cloth_demo/robot_tools/ros_resume_entry.py)、[厂家 ROS 驱动及 piper_msgs](vendor/piper_ros-noetic/)、[piper-sdk 0.6.2](vendor/piper_sdk_0_6_2/)、[健康遥测 wrapper](vendor/driver_with_health.py)。wrapper 是会启动真实驱动的入口，不能当普通离线检查脚本运行。ROS 消息仍需匹配目标环境构建；相机使用独立 RealSense Python 环境。`pyproject.toml` 没有声明完整运行依赖，两个 requirements 文件是历史环境记录，均不意味着跨机器安装后可直接实机启动。

本轮已用 `test_fast*.py` 的假模型、假 ROS/相机验证修改，结果见 [optimization_validation.json](metadata/optimization_validation.json)。[远端历史回归摘要](evidence/piper_right_pick_demo/runs/fast_repair_20261005_182331/full_tests_after_stall_patch_summary.json)中的 426 项是另一轮记录，不是本轮测试数量。离线测试不能代替路径、停止行为或实际抓取资格；历史图片回放也不会生成动作后的视觉后果。
