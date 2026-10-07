# 双臂、三相机实验平台 v0.9（单臂监督动作）

最新实机结果（2026-10-05，优先于下方历史）：**新无隔板笔筒轮已成功：笔已放入杯内、独立松爪并撤离，最终画面确认笔仍留在杯内。** 用户在pick_026后明确“现在对准筒内，可以下降”；pick_027..033固定XY[0.257,0.259]分段下降，gripper_002开至35mm、实测34.37mm。pick_034空爪上抬30mm，53释放/54撤离/55最终三组图均显示笔留在白杯内靠右沿、腕图空爪；final_state.json确认右pose[0.256951,0.258965,0.187258,-0.789290248,1.570796327,0]、arm_status=0/motion_status=0/err_code=0，左臂关节未变且未接收任务命令。grasp_verified=true、release_verified=true、task_success=true。右臂仍使能停在抬离位，目标开口35mm；控制会话94316继续运行，不重启、不再派发。录像98256已STOP/exit0，recording_new_holder/report.json保存front52929/right52928帧且无cleanup error，两路短版与完整录像均已导出并通过全片解码验证。前轮分格杯放置失败、原零位、两次home013监测失败及其后独立确认零位均原样保留；本次成功不追认旧失败，也不构成自动重放或通用停止资格。

本次成功短版（原速、删去等待，各3分29.53秒）：[第一视角](/home/agilex/piper_pen_repeat_video_20261005/first_person_new_holder_highlights.mp4)、[第三视角](/home/agilex/piper_pen_repeat_video_20261005/third_person_new_holder_highlights.mp4)。完整录像（保留等待，各58分48.53秒）：[第一视角](/home/agilex/piper_pen_repeat_video_20261005/first_person_new_holder_full.mp4)、[第三视角](/home/agilex/piper_pen_repeat_video_20261005/third_person_new_holder_full.mp4)。四文件均已验证，[视频报告](/home/agilex/piper_pen_repeat_video_20261005/export_new_holder_report.json)保留区间与异常说明。

最终证据：[笔留在新筒内](/home/agilex/piper_pen_repeat_video_20261005/new_holder_pen_in_cup.png)、[撤离后空爪](/home/agilex/piper_pen_repeat_video_20261005/new_holder_empty_gripper.png)、[本次结果](/home/agilex/piper_pen_repeat_video_20261005/new_holder_result.json)。

新无隔板笔筒轮释放前进度（历史保留，最终成功见顶部）：**已再次夹起笔并移至新笔筒口上方，正在等待用户侧面确认能否垂直下降入筒；尚未下降入筒或释放，task_success=false。** 新run为runs/ros_right_new_holder_20261005，gripper_001目标6mm、实测8.89mm；33_pick_016、34_pick_017及后续抬升画面显示笔随夹爪离桌，grasp_verified=true。当前pick_025实测pose[0.250089,0.252050,0.247060,-0.789290248,1.570796327,0]、q[0.789290248,1.663787469,-0.832818759,0,-0.744190940,0]，arm_status=0/motion_status=0/fault=0，夹爪保持6mm持笔目标。当前唯一控制会话94316，旧32995已退出；两视角录像98256仍LIVE，最新图recording_new_holder/42_pick_025。等侧面答复后再由主控逐步决定低速插入、独立释放和撤离；不要重启/归零/盲重发或提前松爪，录像期间不运行robot_observe争用相机。上一轮落笔失败、两次home013软件监测异常及其后独立确认全零的事实原样保留。

上一轮失败与新归位接管记录（历史保留，当前持笔进度见顶部）：**上一轮夹笔成功，但放筒失败：gripper_002开至35mm后，笔落在旧杯前桌面；不能记为放筒成功。** pick_038空爪上抬30mm正常到位，failed_placement_state.json确认右臂健康、到位、爪宽34.44mm，左臂未动。用户说“应该是碰到隔板弹出了”，这是尚未证实的原因推测。用户随后明确“这次我换了一个没有隔板的笔筒，请你将机械臂归位后重新尝试”：新归位授权覆盖下方旧的无新请求禁止重复归零约束，旧home013失败和独立零位确认仍保留。新尝试使用runs/ros_right_new_holder_20261005和recording_new_holder，由主控按新场景逐段执行；当前控制节点改为session94316，原32995已正常退出；新日志为runs/ros_right_new_holder_20261005/roslaunch_after_home.log，后续只用94316，不额外重启/复位/失能或重放旧目标。本轮home013即使1%仍监测失败，原结果保留；随后237帧/4.719959秒独立反馈确认全零且健康到位，最大年龄22.22ms。人工审查后同冻结入口重新通过3.027秒门槛并零控制帧接管，未重发J0；readopt_telemetry记录六零、accepts=true/failure=null。录像session98256仍LIVE于recording_new_holder。当前没有新一轮成功结论，最终状态以新run证据为准。

上一轮失败放置录像：[第一视角](/home/agilex/piper_pen_repeat_video_20261005/first_person_placement_attempt.mp4)、[第三视角](/home/agilex/piper_pen_repeat_video_20261005/third_person_placement_attempt.mp4)，[导出核对](/home/agilex/piper_pen_repeat_video_20261005/export_placement_report.json)；两路各9分31.47秒，明确记录失败结果，不包含新笔筒尝试。

此前持笔等待记录（历史保留，后续释放失败与新授权见顶部）：**已确认夹起笔并搬至筒口上方，等待用户侧面确认笔尾前后对口；尚未释放，本轮未完成。** 用户恢复照明后，原右臂ROS节点继续执行，没有重启或再次归零。gripper_001开口目标6mm、实测8.54mm；pick_017/018/019/020连续抬升的画面确认笔随夹爪移动。pick_030后最新只读法兰实测pose[0.203887,0.255099,0.197068,3.141592654,1.483529864,-2.245069377]，arm_status=0、motion_status=0、err_code=0，夹爪仍持笔；当前速度5%，后续携笔P命令须保持6mm目标。控制会话32995和日志runs/ros_right_repeat_20261005/roslaunch_after_home.log继续有效。两视角录像会话53970已正常停止、exit0，recording_resume/report.json记录front25224帧/right25223帧、无cleanup error；最新图为25_awaiting_side_alignment，状态为runs/ros_right_repeat_20261005/awaiting_side_alignment_state.json。额外只读左腕相机336222071115未拍到杯或笔，不能解决前后对口问题；没有移动左臂。待侧面答复后另开新录像段，不复位、不重启控制节点、不松爪。当前grasp_verified=true、task_success=false；待现场答复后再由主控逐步审查插入、独立释放及撤离。上轮成功、原始零位、home013监测失败及后续3.82秒独立归零确认均保留，不把它们覆盖成本轮成功。

本轮夹笔至筒口录像：[第一视角](/home/agilex/piper_pen_repeat_video_20261005/first_person_grasp_to_cup.mp4)、[第三视角](/home/agilex/piper_pen_repeat_video_20261005/third_person_grasp_to_cup.mp4)，导出状态见[记录](/home/agilex/piper_pen_repeat_video_20261005/export_resume_report.json)；当前仍持笔，录像不含已确认释放或完成结果。

此前照明等待记录（历史保留，照明已恢复，当前进度见顶部）：**本轮未完成，等待照明恢复和现场看护答复。** 右臂已归零后展开至pick_007；此后两路画面持续变暗，未继续派发，笔仍在桌面、尚未夹取。右臂当前pose[0.238970,0.117128,0.264878,3.141592654,0.872437733,-2.685852279]，arm_status=0、motion_status=0、err_code=0，仍使能；爪宽34.44mm、目标35mm，左臂待机未使能且关节未变。home_001..013请求50%，其中home_013发完4帧后软件名义关节限位监测失败，故障时原始样本缺失、轴和幅度未知；随后192帧/3.820秒独立反馈及同冻结入口新一轮零发送接管确认六零，未重发J0、未改限位。pick_001使用5%，pick_002..007使用50%；这些有限实机段不构成任意50%路径的碰撞或停止资格。当前ROS节点继续运行，日志runs/ros_right_repeat_20261005/roslaunch_after_home.log、会话32995；不要重启节点、重复归零或盲重发点。两路录像已正常结束封装，当前为部分任务录像；继续动作前先取得答复、新清晰画面和新鲜反馈，继续录像则建立新段。详见tasks/put_pen_in_holder/task.json、runs/ros_right_repeat_20261005/session_plan.json和final_wait_state.json。上轮成功与原始零位记录保留，不能用于声明本轮成功。

照明暂停前的部分录像：[第一视角](/home/agilex/piper_pen_repeat_video_20261005/first_person_partial.mp4)、[第三视角](/home/agilex/piper_pen_repeat_video_20261005/third_person_partial.mp4)，封装与验证状态见[导出记录](/home/agilex/piper_pen_repeat_video_20261005/export_report.json)；它们只记录归位和接近阶段，不是抓取放筒完成录像。

上轮已结束结果（2026-10-05，历史保留）：**右臂已完成抓笔、放入白色笔筒左格、松爪并抬离，最终图像确认笔仍留在筒内。** 抓取、移动、释放均经ROS逐点监督执行；独立夹爪请求将目标开至35mm，实测34.44mm，空爪随后上抬29.759mm。最终右臂正常到位、无错误，仍使能停在抬离位；左臂六关节与任务开始时完全一致。见[放笔后图片](/home/agilex/piper_pen_to_holder_20261005/05_pen_in_holder_after_retreat.png)、[撤离后空爪](/home/agilex/piper_pen_to_holder_20261005/06_empty_gripper_after_retreat.png)和[结果摘要](/home/agilex/piper_pen_to_holder_20261005/result.json)。这是无标定条件下的一次监督实机成功，未运行模拟/mock测试，不构成可自动回放计划或通用停止资格。

上轮任务（已完成，本轮重复任务状态见顶部）：**仅用右臂夹取桌上的笔，放入笔筒并释放**。任务由 `configs/robot.json.task_file` 选择，当前为 [抓笔放筒](tasks/put_pen_in_holder/task.json)。原短袖 T 恤对折任务及旧工程记录保留原处。

历史初始位（完整保留）：**右臂六个关节曾实际回到0°，并保存为本次抓笔初始位；上轮结束时停在上述放笔后抬离位置，本轮当前位置见顶部。** [SDK回零结果](runs/home_arm_538edd4ae94943e6be15a6dc8d8e71c2/result.json)为一次1%MOVE_J、4帧，六轴实测均0、到位标志0且稳定3.010秒；左臂及双爪未收目标。右关节与夹爪保持使能、双臂无故障；左臂保持原待机姿态。

[初始位置记录](tasks/put_pen_in_holder/initial_position.json)保存实测关节和法兰位置，并由当前任务引用；没有重设编码器零点。[独立后观测](runs/obs_ddd8a791acc84b429f58f45b5c7a12bd/observation.json)再次确认全零，参考图片存 `/home/agilex/piper_home_zero_20261004_234114/`。第三视角仍可见桌面笔和笔筒；回零后右腕只能看到笔的一部分，后续使用新观测。

抓取中途记录（确认至第037点，现已被上方最终成功结果更新，保留历史）：**右夹爪已夹起笔并抬离桌面，尚未放入笔筒或松爪，任务未完成。** 全部后续写指令经右臂ROS节点，以1% MOVE_P逐点执行。首次10mm开口夹取后出现轻微滑动，独立夹爪请求改为6mm目标；实际开口约9.31mm，随后两次30mm抬升中笔相对夹爪稳定，第三视角可见离桌。第037点后法兰实测Z约187.03mm；第038点目标Z217mm已派发，本次进度记录不将其视为已完成视觉核验。左臂保持被动。接下来仍须对准笔筒、插入、独立释放并撤离确认。见[抓取进度](tasks/put_pen_in_holder/task.json)、[夹爪收紧记录](runs/ros_right_20261005/gripper_004.json)及[抬升后观测](runs/obs_81d02782ff8f4cc38963e79206781176/observation.json)。用户授权无标定尝试，不运行模拟测试。

历史准备状态（抓取前记录，保留）：v0.8增加独立固定单臂回零入口；用户要求取消模拟测试直接SDK执行后，已进入实机并完成上述回零。维护与实机证据见[验证记录](docs/VALIDATION.md)。右J2/J3起点越界已解除；**抓笔放筒尚未执行**，当前相机/工具几何及目标坐标仍待建立，旧双臂任务守卫和一般保持停止资格未被改变。

此前ROS出零准备记录（已由上方抓取进度更新，保留历史）：右臂六零复查通过，右夹爪预开20mm实测19.39mm。随后SDK MOVE_L前移5mm收到控制器状态4，未实动，原失败记录保留。用户要求尝试ROS后，仅启动右can1节点（左臂零动作），以ROS 1%六零关节命令确认当前状态0、到位0、六关节全零。正准备低速MOVE_P单步；尚未抓笔。用户明确允许无标定尝试，不运行模拟测试。

设备绑定：左can0=USB1-6.2:1.0，右can1=USB1-6.3:1.0；第三视角243322070709位于机身对面，两腕序列号不变。移动及更换相机后的旧外参不能沿用。以下叠衣与调试描述保留作历史，当前状态以上述新记录为准。

固定工具层现已包含双臂与夹爪反馈、三路 RGB-D、指定像素深度、厂家 FK、计划预检查，以及调用真实 SDK 的分阶段执行器。
执行器已接入 `move_p` / `move_l`、独立夹爪动作、到位监测、持久化单次提交、独占锁、状态查询和取消请求。
任务模型负责看图、选抓点、选折线、给出阶段目标和判断结果。工具没有叠衣算法、目标检测器或自写逆解，也不自动生成或修正坐标。

v0.3 按用户明确请求增加固定 `robot_request_can_control` 模式接管入口：确认双臂静止、使能、无故障、示教记录关闭后，以 1% 速度参数逐臂请求 CAN 控制并观察反馈；每臂至多一个 0x151，不发送位姿/关节/夹爪目标。模式切换可能产生物理影响，不能称为保证不动；成功也不解除原抓取执行器的停止能力门禁。具体约定见 [工具契约](docs/TOOLS.md)。
v0.4 增加重启后固定 `robot_startup_arms`：双臂待机且全部未使能时，先逐臂确认 CAN 模式，再逐臂厂家使能。每臂最多一帧模式、一帧使能，无任务目标。它复用 SDK 及现有守卫，不新增叠衣算法。

**双臂六关节和空夹爪均已使能，三路相机可直接访问；右臂已实际执行边界恢复和短程直线覆盖试验，左臂尚未恢复，衣服未抓、未折。保持停止资格仍未通过。**
本轮新鲜厂家查询确认左右固件均为 `S-V1.6-5`，与现用 `default` 驱动匹配：[原始帧及厂家解析](runs/firmware_08eb9eb4a47246afaa532c716713169a/result.json)。只发送两帧 `0x4AF` 查询，没有使能或运动目标。查询记录右腕小幅反馈变化；这不证明物理静止，也不解锁抓取。v0.5 增加固定固件查询和空爪准备，后续通用调试入口仍在维护。
2026-10-04 用户确认两臂均为 **PiPER**，配置已从 `piper_x` 更正为 `piper`，项目目录名保留。厂家 FK 与两臂控制器末端反馈吻合；固件另经上述查询确认，工具几何和停止行为仍未完成核验。
用户重启后两臂为待机 0、全部未使能。[本轮启动](runs/startup_3d72f0070d4549f586e6379e0e8c8faa/result.json)每臂各发送一次 `0x151` 和一次 `0x471`，共四帧；双臂新反馈确认模式 1、六关节使能、无故障。随后[空爪准备](runs/gripper_prepare_c6b29caa65ec4327a6f9615587a4ad83/result.json)每爪仅发送一次当前开口目标，双爪使能已确认；没有夹取衣服。先前模式 2 的接管未确认记录仍保留在验证历史中。
[控制器限位查询](runs/joint_limits_dae169c6de344b33b3ab4256bb1a1ec1/result.json)已完成。右臂恢复工具原结果仍为 aborted，随后被动反馈才确认其关节合法且稳定，不能追认恢复全程通过。修正模式与目标之间的等待后，[新直线试验](runs/linear_hold_2657b6880edc4414b4c2bc9dc347ae1d/result.json)以一次完整 `move_l` 发四帧，观察到上移 1.246 mm 后再发三帧覆盖，共七帧；控制器故障 4 清为 0，但十秒内未确认到位，`qualified=false`。[后续三图及状态](runs/obs_51d6d9e1549e4fa8833416fcce182872/observation.json)右臂位置维持在起点上方约 0.364 mm，仍 `motion_status=1`（未到目标）；反馈稳定不证明待执行目标已取消。完整证据见验证记录。
本站姿态保持停止未验证，抓取后端仍硬拒绝，修改配置布尔值不能解锁。上次急停导致下落的暂停记录仍有效；SDK 模拟测试不等于实机验证或碰撞检查。

## 从哪里看起

| 文件 | 用途 |
| --- | --- |
| [机器人说明书](docs/ROBOT_GUIDE.md) | 物理属性、单位、坐标、相机盲区与待核验信息 |
| [SDK 审计](docs/SDK_AUDIT.md) | 哪些是厂家真实计算能力，哪些只是运动后的反馈 |
| [工具契约](docs/TOOLS.md) | 23 个固定工具、输入输出、执行与拒绝边界 |
| [实验提示词](prompts/experiment.md) | 限制任务模型只使用固定工具、提交计划数据 |
| [T 恤对折任务](tasks/fold_tshirt/task.json) | 用户目标、初步阶段意图；未虚构衣物尺寸、折线或坐标 |
| [现场配置](configs/robot.json) | PiPER 型号、设备绑定及 S-V1.6-5 固件已确认；坐标、工具几何和停止能力仍有待核验项 |
| [验证记录](docs/VALIDATION.md) | 283 项离线测试，以及单独记录的实机调试结果 |

## 关键发现

新版 SDK 有纯 FK：关节角 → 法兰位姿。`get_ik_joint_angles()` 则是在特定固件、提交 `move_p` 之后读取逆解反馈，不能用于无运动试算。
`move_p` / 旧 `EndPoseCtrl` 可以将笛卡尔目标交给厂家控制器逆解和执行；Python 返回并不等于目标被接受或已到位。

新版 `piper_x` 与 `piper` 的零位计算结果不同，历史右臂零位及当前双臂反馈均吻合 `piper`；用户已独立确认两臂为 PiPER，配置疑点已纠正。
当前预演只检查数据、观测依据并绘制阶段意图，明确列出未完成的 IK、路径和碰撞检查。

## 本地入口

使用现有 Python 环境，不安装或改动 SDK：

```bash
cd /home/agilex/piperx_cloth_demo
/home/agilex/miniconda3/envs/pi0_infer/bin/python3.10 -B -m robot_tools.server --call robot_describe
```

该命令只显示配置和能力，不连接设备。MCP stdio 入口为：

```bash
/home/agilex/miniconda3/envs/pi0_infer/bin/python3.10 -B -m robot_tools.server
```

stdio 入口由兼容客户端启动并通过标准输入通信，直接运行后等待输入是正常现象。
[连接配置示例](configs/mcp.example.json) 未写入现有客户端设置，也没有自动接入本次对话。
实现的是 MCP **2025-06-18** 的 initialize、ping、tools/list、tools/call 子集，未宣称支持所有版本或已通过真实客户端联调。
它使用当前进程权限，不提权、不改变 CAN、不绕过会话沙箱；当前会话权限不足时，换成 MCP 名称也不会自然获得设备权限。

在具有正常设备权限的终端，可用同一固定工具只读采集一次（不使能、不运动）：

```bash
cd /home/agilex/piperx_cloth_demo
/home/agilex/miniconda3/envs/pi0_infer/bin/python3.10 -B -m robot_tools.server --call robot_observe
```

采集会保存前视、左右腕 PNG、可选深度、双臂反馈及时间信息到新建 `runs/obs_*`；部分设备失败时也保存报告。
通过 MCP 调用时图像会作为图像内容返回给模型，CLI 则输出文件路径。不会把失败或缺失状态补成正常值。

## 第一次对折怎样推进

1. 获取新的三视角和两臂状态，确认 T 恤是否平铺、尺寸范围、折线和两个可用抓点。旧方块图像不适用。
2. 核对现场型号、基座与工具参考点；处理 SDK 能力和停止策略的缺口。
3. 模型一次提交“接近、抓住、提起、跨折线、放下、释放、撤离”的阶段计划；每个阶段写清预期反馈及不确定性。具体抓点与顺序随画面确定。
4. 固定工具返回结构问题和能力缺口；模型针对具体原因修订计划数据。未知项不能通过不断改坐标绕开。
5. 验证现场停止能力及其余运动前提后，再接通已经实现的 SDK 执行器。完整计划不等于盲执行：反馈监测仍持续，异常时不再派发后续动作，也不自动急停、失能或回零。

当前已取得完整三相机及双臂观测，尚未生成可执行的叠衣坐标。
模型的[衣形判断与完整阶段意图](runs/obs_4a4d7d907ada43d2bb001b6e30d89a48/agent_assessment.json)已保存：两手各抓同侧下摆角，向肩部方向横向对折；整条正常流程一次规划，在轻提和释放后核验衣物状态。后续机械臂调试已改变右臂起点，须使用上述新观测及派发前新鲜反馈，不能沿用该衣形判断时的机械臂姿态。
`paired` 只保证尝试近时派发及等待双方完成，不是原子发送或同步轨迹。夹爪当前只按目标开口到位判断；布料阻挡使开口达不到目标时会超时，不会被自动解释成抓取成功。
提交为同步调用。取消需要另一进程提交请求或 Ctrl-C；请求收到、停止后续派发、物理停止是三件事。进程退出或通信断开不保证机械臂已停止，详见工具契约。

## 参考 GPT6-ARX5 的范围

实机项目仍是本目录。`/home/agilex/GPT6-ARX5` 提供另一套硬件上的通用图像引导方法，本地尚未找到叠衣专用流程或成功记录。
参考其[模型目标与工具链](/home/agilex/GPT6-ARX5/PROJECT_HANDOVER.md:56)和[无外参图像引导方式](/home/agilex/GPT6-ARX5/PROJECT_HANDOVER.md:96)：三路图像与双臂状态交给模型，模型给末端目标或整段路径点，固定工具负责检查、厂家求解、派发及反馈。任务变化应修改模型计划数据，不新增任务算法。
ARX 的离线 R5 逆解、时间协调与驱动并不自动存在于 Piper 工具中；其坐标、夹爪单位、初始化、回位和停止操作不能照搬。本次只参考方法，不运行 ARX 脚本、不移植硬件代码、不把其叠杯成绩称为叠衣验证。

昨天抓方块的成功控制链同样应复用。[实际完成记录](/home/agilex/piper_right_pick_demo/runs/pose_batch_20261003T213623_137631/report.json)已经采用厂家控制器逆解，运行时外部 IK 调用为 0；其使能后逐驱动新反馈确认、`MotionCtrl_2 + EndPoseCtrl` 和独立夹爪命令都是通用接口。该轮起点已 CAN 控制且已使能，旧计划只针对右臂方块；不能把旧坐标用于当前衣服，也不能把停止后续派发当作已取消正在执行的目标。详见 [SDK 审计](docs/SDK_AUDIT.md)。

## 维护与实验分开

本次允许编写的是可复用 SDK/相机适配器。后续实验应冻结它，只允许模型生成计划 JSON，不允许模型临时编写检测、轨迹或控制程序。
提示词只能提醒；实际实验客户端还应只暴露这里的固定工具，不提供 shell、文件修改、任意 Python 或网络执行工具。
需要新底层能力时结束该次实验，记录缺口，再进入维护阶段。不能把“调工具参数”扩展成临时改底层代码。

`PLATFORM.sha256` 和 `SDK_SOURCE.sha256` 保存本轮文件快照，供维护时检查变化；它们不是运行时防篡改机制，也不自动锁定系统权限。
无设备测试：

```bash
/home/agilex/miniconda3/envs/pi0_infer/bin/python3.10 -B -m unittest discover -s tests -v
```


## v0.1 复用分析讨论历史（保留原文）

以下是用户添加的早期讨论，保留用于比较；其中“执行尚未接通”和固定任务路径描述对应 v0.1。v0.2 已有 SDK 执行实现及 `task_file` 配置，但本站实机仍暂停。

当前叠衣服项目：**大部分可以复用，换任务不会让整个工程失效，但当前版本还没有彻底做到“与任务无关”。**

这些代码主要是在建立固定工具接口，并没有编写“如何识别 T 恤、选择折线、生成叠衣轨迹”的算法。

| 部分 | 换任务能否复用 |
|---|---|
| SDK 连接、读取机械臂状态、厂家 FK | 同一套机械臂基本直接复用 |
| 三相机采集、图像和深度返回 | 基本直接复用 |
| 工具调用协议、参数检查、记录保存 | 可以复用 |
| 机器人说明书、坐标单位、设备限制 | 硬件和安装没有变化时可以复用 |
| T 恤任务描述、折叠阶段意图 | 需要替换 |
| 实验提示词 | 通用约束保留，叠衣相关内容替换 |

一些任务耦合：例如 piperx\_cloth\_demo/robot\_tools/service.py 固定读取 `tasks/fold_tshirt/task.json`，提示词也包含叠衣内容。这里应该进一步整理成“固定机器人平台＋可替换任务配置”，让切换任务不需要修改 Python。

期望的使用方式应当是：

**换任务描述 → 模型看当前画面 → 模型给出完整计划 → 同一套工具检查并执行 → 返回反馈。**

例如从叠衣服换成搬方块、摆放物品，只要已有工具覆盖所需动作，就应该仅更换任务和计划数据。若换成擦桌、插接头等需要接触力控制的任务，现有位姿工具可能不足，需要增加通用的力控能力；增加的是底层能力，不应重新写一套具体任务程序。

还需要明确一个边界：**当前工程已经具备的是观测和计划检查工具，实机执行尚未接通。** 因此，现在不能说它已经是能执行各种任务的通用机器人平台。

后续工作：**换一个已有动作能力能够覆盖的任务时，机器人平台代码是否保持不变，仅由模型改变任务计划。** 我们应该围绕这个标准继续整理和验证。
