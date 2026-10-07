# 固定机器人工具

`robot_pair_*` 工具提供独立的持续连接有界串行执行器：同一宿主持有两臂连接，每次只动一臂，另一臂持续读取反馈且禁止 TX。接口、持久账本与生命周期见 [持久双臂有界宿主](PAIR_HOST.md)。通用 `astra` recipe 仍为离线合同；接触支撑和实际停止尚未实机验证，软件测试不授予拔插资格。

新增 `kind=joint` 的唯一目标字段为六轴 `target_joints_rad`，与末端位姿和夹爪开口互斥。当前接空载 approach/align，以及同宿主确认释放后、每段有新空爪 RGB 描述的 release_retreat；生产来源读取器、独立首次 MOVE_J 初始化和来源绑定的边界内移已接入，实际现场几何仍需补齐。缺来源会在派发前明确拒绝。调用者不能传入 context、几何证据或权限布尔值。显式 `robot_pair_cancel` 在完整原 MOVE_J、同工作线程/连接且反馈适用时，可领取一次独立计步的有界同模式保持；未知/部分发送、普通故障和 EOF 不走该例外，后续独立故障终止保持。实测保持、原目标取消和物理停止分开报告，MOVE_L/夹爪仍沿用软件锁存。详细范围见 [实现设计](PIPER_X_JOINT_PATH_DESIGN.md)。

普通 joint 内移只接受同 owner/连接、当前完整缓存及全套有效限位对应的完成初始化来源。仅 J2 下界/J3 上界的最多 0.003 rad 观察尾差有此分流；名义限位、目标内侧 0.010 rad、单轴 0.025 rad 和整臂净空保持不变。固定 X 模型的新法兰区间界仍计入原 hold 余量，但原始双臂参考有尾差时 `hold_reference_within_nominal_limits=false`，现有 hold constructor 仍拒绝。MOVE_L 任意发送尝试使该侧旧 joint 来源失效，夹爪动作保留来源。候选/静态夹持的受支撑释放现已接通；带载动作、RGB 物体进展及目标终态仍未接通。内移离线证据见 [对应记录](../../../artifacts/plug_ingress_1791356964575384710/)。

现有状态快照附带可选 `motor_feedback`：六个电机分别报告电流 A、速度 rad/s、位置 rad、SDK 估算力矩 Nm 和原始接收时间/新鲜度。只复制 RX 缓存，不查询、不增加必需反馈分片；缺失或过期时不阻断原本适用的基础动作。它不是腕部力传感器，不产生夹牢、带载支撑或插入力上限结论，故障后的只读反馈也保留这些诊断。

v0.9新增 `robot_single_arm_move_once(arm,target_pose_m_rad)` 与 `robot_single_arm_gripper_once(arm,width_m,nominal_force_N)`。与原监督单次的目标/速度边界相同，区别是另一臂可维持原模式0/1/2及已知七使能位，不需为了单臂任务使能它。选中臂和夹爪仍须已使能；关节反馈按下述有界观察策略判断，名义越界如实保留。另一臂名义越界只记录、全部发送封锁。双方新鲜健康、示教关闭、起点到位且三秒稳定；移动中只有选中臂可报告未到位。独立年龄上限100ms（包含本地校验/日志耗时），静止姿态补SO(3)检查。分别保存`runs/single_supervised_move_*`或`runs/single_supervised_gripper_*`，与旧工具及完整计划保持停止门禁分开。ok仅代表单次派发和有限稳定观察完成；必须另看原始到位、末端/开口误差和图像，不代表抓住。用户已明确要求无标定尝试，只允许有当前图像依据并标明不确定性的逐段监督尝试。

2026-10-07 用户要求处理严格边界误拦：仅 `robot_single_arm_*_once` 使用新增反馈策略。普通反馈允许超出名义边界至多 `0.003 rad`（约 0.172°）；这是软件观察带，不是厂家精度、物理安全或目标关节路径认证。夹爪动作不发送机械臂目标，可额外保留起始 J2 低于下限或 J3 高于上限、各不超过 `0.1 rad`（约 5.73°）的静态偏差；该上限是本轮软件维护选择，不是零点标定。首次健康反馈冻结偏差基准，后续样本不得重建或抬高该基准，并继续执行三秒稳定、原始关节/位置/SO(3)漂移守卫。此策略仅约束单次调用，没有持久化跨调用偏差基准；不能将连续调用当作偏差逐步扩大的许可。

结果独立返回 `selected_arm_strictly_within_limits`、`selected_arm_within_feedback_tolerance` 和 `static_boundary_exception_accepted`，不会把容许观察写成名义合法。超出本动作策略时，零发送结果含 `boundary_recovery_required` 和最近名义边界候选；候选不能自动执行，另列现成恢复入口的 0.05 rad 单轴步长是否满足，FK/协议量化/整臂净空等仍须核对。发送后异常不生成恢复候选。MOVE_L 仍没有目标关节逆解/路径证据，几度的起始偏差不会由这个夹爪例外放行移动。控制器/SDK 限位、旧双臂监督入口、home、速度、发送白名单和故障处置保持原合同；recovery 默认合同保持，显式启动 profile 见下文。

v0.8新增 `robot_home_arm(arm)`，仅接受left/right，固定六零关节目标和1%速度，不接受目标、设零或重试参数。起点限各轴绝对值[45,10,10,15,30,15]°，J2/J3可从名义边界外最多5°向既有零位返回；其它起点/目标名义范围保持。选中臂模式1、六关节使能，另一臂保留模式0/1/2及七使能位，另一臂（本次左臂）与双爪完全禁止发送。

起点须三秒稳定、至少20次完整反馈推进，全部反馈在校验完成时年龄≤100ms；仅一次厂家MOVE_J连续发送0x151和0x155/156/157，后三帧目标全零。途中监测各轴初值至零的独立区间加0.01rad跟踪量，原始边界外读数不得加深超过0.003rad，实时FK与法兰一致；被动臂途中位移限制2mm，基线与最终稳定窗口位移跨度均限制0.5mm；这不是碰撞/路径认证，调用者必须结合现场看护检查整臂和附件扫掠。完成须所有片段新于发送、模式J、到位0、六轴误差≤0.003rad、零目标FK误差≤2mm/0.01rad并稳定三秒。最多观察120秒；任何失败不重试/补发/停止/复位/失能。已有零位容差内时只观察，不为切模式额外发令。结果独立报告近零与严格范围合法，不自动授予抓取资格。保存 `runs/home_arm_*/`。

工具实现连接 SDK、读设备、检查数据、执行固定动作并保存记录。没有衣物分割、红块检测、自写 IK 或自动轨迹生成。
计划由模型提供；`robot_preview_plan` 保存原计划与 SHA256，不重新写轨迹。
完整计划 SDK 后端仍因本站保持停止未验证而硬拒绝，配置文件不能解除这个限制。用户已授权有限监督尝试及通用缺口维护，当前任务以 `robot_describe.task` 和用户最新要求为准（2026-10-04 改为单右臂夹笔放入笔筒）；单次运动/夹爪入口须经维护审查和冻结后使用，不能把工具实现或离线检查称为实机资格通过。

v0.3 的独立模式接管是用户授权的控制器准备操作，不能作为绕过抓取运动门禁的入口。它不接受目标、使能或停止参数；仅允许单次明确模式帧。模式切换可能影响旧目标的响应，即使没有新目标也不能保证无运动。调用前须明确获得用户的模式接管授权并检查现场。
v0.4 的独立启动入口适用于用户重启后的待机、未使能双臂。它复用模式守卫，再调用厂家关节使能；不增加任务算法。初态不匹配、部分启动或不确定结果均不能自动重放。

v0.6 按用户“仅右臂，参考旧启动方法自行完善并重新使能”的明确要求新增 `robot_startup_arm`。仅选中臂必须待机0且六关节与夹爪全部未使能；另一臂允许原控制模式0/1/2及已知使能位，持续健康、静止监测，不被接管、使能或发送任何帧。仅用于明确授权的单臂启动，不改变任何已有动作守卫，不用于修复限位或解锁原后端。原双臂启动语义不变。

v0.7 新增独立 `robot_prepare_gripper(arm)`，用于一只空爪按实测当前开口初始化。原双爪准备的5–70mm合同保持；新入口只接受臂名，不接受目标开口，允许实测当前开口0–70mm并固定名义力0.2，不能拿它张爪、闭爪或抓物。该范围是有限准备输入范围，不是现场行程标定。选中臂须CAN模式且六关节已使能；另臂保持原模式/七使能位，只读监测并封锁全部发送。关节范围外读数保留，不授予臂运动资格。需要空爪、无接触及现场看护；当前位置目标与使能同帧，仍可能微调夹指。


### 非零起点的显式边界恢复

`robot_recover_joint_boundary` 的 `recovery_profile="startup_j2_j3"` 针对启动时 J2 低于下界或 J3 高于上界的空载情况；最大回归幅度固定 0.10 rad（约 5.73°），不是对厂家运动范围的修改。六个目标由调用者给出，越界轴须回到最近精确边界，其他轴相对起始与新反馈最多 0.003 rad；目标不随观察自动修正。普通 MOVE_L 和默认 standard 恢复限制不变。

此 profile 的 `attachment_radius_m` 必须覆盖两臂从法兰到夹爪、相机、支架、线缆等最远点。`available_clearance_m` 是外界、另一臂和非相邻自身实体的当前最小表面净空，排除本就相连的铰接/刚性安装面；由调用者建立，工具不从 RGB 推定。厂家 MDH 剩余链长、60 mm 壳体余量、全部六轴目标变化与 0.003 rad 跟踪带构成 sweep；相对位移取 `2*max(sweep, passive_sweep)`，必须小于净空减 5 mm。小于 15 mm 的 FK 终点检查仍独立执行，不能代替整臂通道。

两臂六关节均须已使能。两爪允许已知失能但必须空载、无接触、健康且开口稳定，使能位冻结，不发夹爪或被动臂指令。三秒基线及至少 20 次完整反馈推进、发送前 motion_status=0、包括 FK/日志处理后的 50 ms 反馈年龄均核对；固定 1% 一次四帧 MOVE_J 后观察三秒稳定。微小终态残差保留 `selected_arm_strictly_within_limits=false`，另报 `selected_arm_within_feedback_tolerance`，不据此宣称校准或任务资格。

四帧不是原子提交：模式帧可能激活旧缓存，半帧可能混合旧目标，以上包络不覆盖这些未知路径。保持现场看护、部分发送记录、零重试；退出/断连不证明停止。角度适用或恢复成功不自动解锁任务，更不代表拔插已经完成。诊断候选会给出 `within_startup_recovery_angle_contract`，`startup_recovery_ready` 仍为 false，直到该操作完整合同逐项满足。

## 工具表

通用工具不生成叠衣目标。写操作共享独占锁并先保存请求及发送意图，再保存反馈和结果；锁不能约束平台外控制程序。

| 工具 | 输入 | 输出和副作用 |
| --- | --- | --- |
| `robot_describe` | `{}` | 配置、任务、厂家 SDK 能力及说明书/审计/实验约束正文；不访问硬件，不需另开文件工具 |
| `robot_read_state` | `{}` | 两臂关节/法兰/状态/六驱动器与夹爪反馈、分片时间及健康检查；不发送 CAN |
| `robot_request_can_control` | `{}` | 双臂通用 CAN 控制接管；1 秒稳定起点、逐臂至多一个 `0x151`，保持当前 P/J/L 选择、1% 速度参数，每臂切换后观察 2 秒。已是 CAN 模式则不重发；异常不自动重试或派发下一臂。保存 `runs/takeover_*/request.json`、`events.jsonl`、`result.json`，不解锁运动 |
| `robot_startup_arms` | `{}` | 要求初态两臂待机 0、六关节及夹爪全部未使能、无故障且稳定。先逐臂发送一次 `0x151` 并确认模式 1、仍未使能；两臂模式均确认后再逐臂一次 `enable(255)`（`0x471 0702000000000000`），依新驱动反馈确认使能。保存 `runs/startup_*/`，持续检查两臂漂移、状态和通信。无目标/回零/停止/失能/重试。厂家全电机使能可能影响夹爪使能位，报告实际位且检查开口变化；不发送 `0x159`，不宣称夹爪或抓取就绪 |
| `robot_startup_arm` | `arm`：left 或 right | 选中臂从待机0、六关节与夹爪全未使能的健康稳定起点，一次低速 `0x151` 确認 CAN 模式且仍未使能，再一次 `enable(255)` 的 `0x471` 并核对发送后六驱动反馈。双臂持续读取；另一臂控制/运动模式及七个使能位保持，全部 TX 封锁。复用1秒起点稳定、2秒阶段观察、0.003 rad关节/2 mm位置与开口漂移边界，单臂新入口另核对0.003 rad环绕RPY漂移。无目标、夹爪准备、停止、复位、失能或自动重试；保存 `runs/single_startup_*/`，仅证明本次启动反馈，不修复关节范围或授予抓取许可 |
| `robot_inspect_firmware` | `{}` | 每臂一次 `0x4AF/DLC1/01`，核对新鲜原始响应及厂家解析；漂移留证，不改配置或授予运动许可 |
| `robot_inspect_joint_limits` | `{}` | 每臂每轴一次 `0x472` 查询，核对 `0x473` 角度上下限；不设置限位。速度注释与解码比例有差异，不据此许可运动 |
| `robot_prepare_grippers` | `{}` | 确认空爪后按当前开口、名义力 0.2 各至多一次 `0x159` 位置使能。右 J4 观测容差 0.008 rad，其余轴 0.003 rad；不证明抓住或保持停止 |
| `robot_prepare_gripper` | `arm`：left 或 right | 选中空爪按当前实测开口0–70mm至多一次`0x159`，名义力0.2、使能1、置零0；不接受目标开口或力度。选中臂六关节须已使能且CAN模式，另一臂原模式0/1/2及七使能位保持且零TX。双臂严格0.003rad关节漂移与SO(3)整体旋转差、2mm位置/开口漂移，预发送目标距新实测≤0.5mm，发送后新反馈确认使能及开口误差≤1mm。已使能只观察；无臂目标、模式、复位、失能、停止或重试。保存`runs/single_gripper_prepare_*/`，不授予抓取或保持停止资格 |
| `robot_recover_joint_boundary` | arm、target_joints_rad；可选 recovery_profile、attachment_radius_m、available_clearance_m | 默认 standard 最大 0.05 rad；显式 startup_j2_j3 允许 J2 下界/J3 上界最多 0.10 rad 的空载起点，需要附件与净空参数。最近边界目标、1% MOVE_J 四帧；不是标零，不自动重试或故障停止 |
| `robot_bounded_joint_step` | arm、target_joints_rad、attachment_radius_m、available_clearance_m | 仅 J2/J3 可变，各至多 0.025 rad，编码目标至少离限位 0.010 rad；其它轴与新反馈编码相同。厂家 FK 位移至多 15 mm；MDH 固定起点扫掠加六轴 0.003 rad、附件及 60 mm 壳体余量，须小于净空减 5 mm。一次 1% MOVE_J，实际到位及稳定 3 秒才成功；不是任务轨迹或停止资格 |
| `robot_qualify_linear_hold` | arm，可选 prior_mode_only_run_id | 固定 6 mm 上移、一次目标覆盖的局部试验；来源绑定的受限历史模式错误分支见 schema。不得据此宣称一般停止能力或解锁后端 |
| `robot_move_once` | arm、target_pose_m_rad | 模型审查整段路径后，一次厂家 MOVE_L，固定 1%，相对新反馈平移至多 30 mm、旋转至多 0.05 rad。两臂须健康、合法、新鲜且实测静止 3 秒。分别报告稳定、原始到位状态及误差，不自动接续 |
| `robot_gripper_once` | arm、width_m、nominal_force_N | 同样先确认双臂健康、合法、新鲜并实测静止 3 秒；单爪目标 0–0.055 m，名义力固定 0.2，一次 `0x159`，无臂目标。接触、闭合或开口稳定均不证明抓住 |
| `robot_observe` | `include_depth` 可选，默认 true | 三视角 RGB、可选对齐深度、随后读取双臂，保存 observation_id 和文件；不运动 |
| `robot_depth_at_pixels` | observation_id、camera、1–32 个整数 `[u,v]` | 读取模型所选 RGB 像素对应的米制深度及采集年龄；无效返回 null，无检测/插值/坐标转换，不访问设备 |
| `robot_fk` | 型号、6 个 `joints_rad` | 厂家 FK 输出法兰 m/rad、名义关节越界列表；不建总线、不求逆解 |
| `robot_preview_plan` | `plan` 对象 | 格式/依据检查、原计划、阶段意图 SVG、preview_id；始终 executable=false |
| `robot_submit_plan` | `preview_id` | 校验原计划及当前前提；本站保存 blocked、零派发。已实现的执行路径调用厂家 move_p/move_l/夹爪并监测到位 |
| `robot_check_execution` | `{}` | 不访问设备，列出部署与停止能力缺口；不授予运动权限 |
| `robot_execution_status` | execution_id 或已知 preview_id，二选一 | 读取持久化结果或最后事件；运行/中断状态不明时不自动续跑 |
| `robot_cancel_execution` | execution_id 或已知 preview_id，二选一 | 保存取消请求；响应不是已停证明，不使用快速急停/失能/复位兜底 |
| `robot_pair_open` | run_id、task_id、workspace_clearance_statement；可选预算和 connection_mode | 长期 stdio 唯一宿主；默认 ready 保留已就绪条件，prepare 零 TX 连接并观察未准备状态；冻结原预算，不接受一次性 `--call` |
| `robot_pair_prepare_gripper` | event_id、observation_id、arm、empty_jaw_observation | 当前空爪图像语义与同连接反馈下，至多一次实测开口 0x159；不接受宽度/力度参数，不自动升级任务就绪 |
| `robot_pair_inspect_joint_limits` | event_id | 同连接逐臂逐关节至多十二个 0x472 查询；保存独立原始回复，完整成功才安装当前限位来源，无运动或配置帧 |
| `robot_pair_initialize_joint_target` | event_id、observation_id、arm、unloaded_observation | 同连接、原预算的一次空载当前位置或最近 J2/J3 边界初始化；目标与数值来源由宿主生成，准备态可调用。完整四帧、新 J 反馈及三秒到位后才建立缓存，已有缓存零 TX 返回原记录。部分发送锁存，不重试；不直接授予带载或普通动作资格；边界尾差由普通入口另核当前初始化来源 |
| `robot_pair_promote_ready` | `{}` | 零 TX 核对原任务条件，跨准备保留原姿态/模式基线；缺条件返回 preparation_required，不重连或自动使能 |
| `robot_pair_observe` | rgb_observation_path | 读取现成三路 RGB 元数据与新机器人反馈，生成共同场景及另一臂独立回执；不开相机 |
| `robot_pair_publish_geometry` | observation_id、record_set_id | 从固定本地目录导入实际安装、工作区及当前净空记录，推导保守边界并绑定原 owner/连接/场景；零 TX，不接任意数值/路径/资格布尔值，不采集测量或放行动作 |
| `robot_pair_submit_once` | event_id、observation_id、peer_receipt_id、arm、kind、operation 及唯一目标；probe 可带 grasp_object_id；释放/撤离语义参数见下文 | 持久 claim 后异步尝试一次；`grip_supported` 夹爪至多 5 mm 闭合观测；有身份的有界开爪到位只记 release_opened，确认分离后可接空爪 joint 撤离；不授予带载拔插资格 |
| `robot_pair_retain_grasp` | event_id、episode_id、observation_id、当前 RGB 视觉描述及物体/支撑关系 | 新三秒反馈与原目标、锚点、期限建立静态保持；另一空臂可准备或按当前确认记录撤离，机器合同不接受模型传入 |
| `robot_pair_confirm_release` | event_id、episode_id、observation_id、visual_description、object_relation=`object_clear_of_fingers`、support_relation=`independent_support_present` | 最后开爪后的新保存 RGB 与适配器新三秒 trace，持久确认后零 TX 清理本地未决状态；不证明目标孔正确、终态稳定或停止 |
| `robot_pair_status` | event_id 可选 | 查询宿主或动作回执；动作期间可响应，物体成功与停止分别报告 |
| `robot_pair_cancel` | reason | 锁存整对软件故障，阻止后续目标帧；不证明物理停止 |
| `robot_pair_close` | `{}` | 无未决动作时关闭资源；带未解除接触候选关闭会持久锁存。正常 detach 可按原 run 剩余预算接续，故障不清除 |

严格输入定义在 [service.py](../robot_tools/service.py) 的 `TOOL_SCHEMAS`，MCP `tools/list` 返回同一份定义和详细工具描述。
未声明字段、非有限数、错单位形式、重复臂、跨臂参考系错误都拒绝；没有执行字符串或任意文件路径参数。

有身份的候选/静态夹持每次 `kind=gripper, operation=release_retreat` 须提供当前 RGB 的 `release_support_observation` 及 `release_support_relation="independent_support_present"`。每次增加最多 5 mm，机械到位后仍为未决 `release_opened`，可新图后继续开爪，再调用确认工具。确认后的每段 `kind=joint, operation=release_retreat` 须带 `release_retreat_observation` 描述当前空爪；新夹爪 claim 使宿主确认 token 失效。未绑定物体的旧 probe 只做机械残余清理，v1 `released` 不自动获得 v2 的分离语义。完整边界见 [受支撑释放流程](PAIR_HOST.md#受支撑释放与空爪撤离)；本轮未验证实物释放，也未开放 loaded 状态。

模式接管不使用 `set_speed_percent()` 的 MOVE=255 报文，也不裸调用带默认 50% 缓存的 `set_motion_mode()`。它在 SDK 模式缓存中明确设定审查过的字段，再调用厂家 `set_motion_mode()`；此处依赖已审计 SDK 内部 `_msg_mode`，SDK 升级需要重新审查。最终总线发送字节必须匹配白名单，CAN send 成功还需新状态反馈验证。检测到漂移、故障或通信失败后，不用快速急停、失能、复位或当前目标覆盖作兜底；断开连接也不是停止证明。与轨迹执行共用独占锁，但该锁不能阻止平台外的控制程序。

监督单次用于本轮已授权且有人看护的正式尝试：模型先形成当前任务完整阶段计划，再逐段审查现场和完整运动空间，调用一次并检查反馈；不允许任意批量派发或重放被拒绝的整份计划。数值步长不是碰撞、IK 路径或停止证明；失败只停止后续软件派发，不自动重试、stop/reset/disable，机械臂可能仍在执行。`ok` 须结合 `observed_stable`、原始 `motion_status`、位置/开口误差及任务物体画面解读；稳定但未到位也可能返回 `ok=true`，夹空不得称为抓住。单右臂任务仅指定 `arm=right`，左臂仍被监测且必须满足原守卫，不收到任务目标。

现场依据：用户确认附件范围 30 cm、两臂约 5 cm 净空、桌面约在基座下 1 cm；这些不是标定或接近桌面时持续成立的净空保证。下摆悬出 2–3 cm 的观测为 `obs_335431aff99349299bcd46e746bb178c`。右 `bounded_joint_step_f0776e75156042728d76580d2205320d` 已实测到位并稳定 3 秒；左 `joint_recovery_fad73c4df52b4fb0a695be5dbf4dbf7f` 超包络中断，后续观测才显示合法到位，原试验不得改为通过。一般保持停止仍未验证。

## 计划数据约定

- 顶层：`schema_version: 1`、`task`、由工具生成的 `observation_id`、`stages`（1–24 阶段）。
- 阶段：唯一 `id`、`intent`、`expected_feedback`、`coordination`（`sequential` 或 `paired`）、1–2 个 `targets`。
- 目标：`arm`、与臂匹配的 `frame`、`reference: sdk_flange`、6 数 `pose_m_rad`、`motion`、`speed_percent`、`uncertainty`；显式写 `action=move` 或 `action=gripper`。兼容旧计划时省略 action 按 move 处理。
- `pose_m_rad = [x,y,z,roll,pitch,yaw]`；各自基座、m/rad。roll/yaw 在 ±π，pitch 在 ±π/2；越界直接拒绝，防止 SDK 静默截断。
- move 动作只选 `move_p` / `move_l`，不带夹爪字段；直线指令不意味着所有臂体沿直线或已避障。
- 本版参数速度上限为 5%，只是输入约束，不是安全速度证明；控制器实际速度仍需现场验证。
- gripper 动作必须同时给 `gripper_width_m`（0–0.1 m）和 `gripper_force_N`（0.001–5，厂家文档单位 N）。这是工具输入范围，不是已核验行程或安全力度建议。
- gripper 的 pose 是应保持的当前法兰位姿；当前通用 schema 仍保留 motion/speed 必填字段，但夹爪动作不派发臂运动。移动和开合必须拆成不同阶段；同阶段不混合两种 action。
- `paired` 要求两个不同的臂，尝试近时派发后等待双方完成；`sequential` 按目标列表逐个派发、逐个等到位。双臂及单臂多帧 CAN 都不是原子事务。

完整 JSON 结构由工具 schema 提供，此处不放可误执行的示例坐标。当前抓笔任务及历史 T 恤数据中的阶段意图不是可执行 plan；没有坐标依据时保持 `numeric_motion_plan=null`。

## 时间与完整性

双臂只读适配器先核对所有 CAN 的 USB 绑定，再接 SDK；发送路径封锁，不自动 enable/query/reset/stop。
每臂分别检查 3 组关节、3 组法兰、臂状态、6 个驱动器及夹爪片段，最大年龄 0.5 s，片段及跨臂时间差上限 0.1 s。
这些片段不是原子快照。收到完整数据、健康检查通过、获得运动许可分别报告；执行层会重新核对年龄、故障与使能状态。
清理失败、通信异常或缺帧都降为部分/失败。接入时按当前配置核对物理 USB 绑定，不能仅凭 can 编号推断角色。2026-10-04 22:40 已核对左 can0=1-6.2:1.0、右 can1=1-6.3:1.0；当前 can1 是原 can2 右从臂的重新枚举，不能与更早的 can1 示教主臂混淆。

三相机逐一绑定指定序列号，640×480、15 fps；启动后预热、清队列，限定主机接收时间差，失败有界重试并释放设备。
深度是 RealSense 对齐到彩色的 float32 米值，原始 0 / 65535 转 NaN，并记录有效性统计；旧文件的 65.535 m 编码上限在取样时也视为无效。采集完整不等于深度可用，不变换成机械臂坐标。
主机收帧时差不等于曝光同步；跨设备时钟未校准。组合观测是先取相机后取机械臂，明确报告顺序与总时间范围。
观测超过 30 s、缺设备或未找到时会报告 blocked；30 s 只是本阶段草案检查阈值，不能作为未来动作前安全新鲜度标准。
图像作为 MCP image 返回，可让模型直接观察；完整文件与部分失败保存在独立的新目录。

## 拒绝与修订

输入错误返回具体字段和原因；模型只修订相关计划数据。已保存计划被改动时，旧 preview_id 不可复用。
配置真假声明不等于已执行测量。即使把所有 verification 改成 true，真实后端仍因本站 hold 未验证而拒绝实机派发。
求解缺失、碰撞未查、停止未验证分别列出；没有“所有检查通过”这样的笼统结论。
厂家 IK 在提交笛卡尔目标后由控制器执行，不是目前预检查工具已经完成的步骤；厂家的错误与实际位置必须共同判断。

## 执行、取消与恢复

完整计划执行器检查起点漂移、连续新鲜反馈、目标误差和稳定时长；移动期间夹爪不得意外变化，夹爪动作期间臂不得移动。
当前默认到位容差为位置 5 mm、姿态 0.05 rad，稳定 0.3 s；夹爪为开口误差不超过 2 mm。它们是判断阈值，不是精度或安全保证。
夹爪只支持目标开口到位，未实现“受衣物阻挡即视为夹住”的完成条件。衣物接触但达不到目标会超时；`grasp_verified` 不会因此自动变成 true。
派发前持久记录意图；进入执行后的 preview_id 只能领取一次，同 ID 重试不重放。平台跨进程锁防止本平台并发执行，不能约束其他 CAN 控制程序。
运动中保存反馈和事件；部分派发、超时、取消或故障后停止接续，只请求经过验证的保持。不能补发、回零、反向轨迹或清故障来自动恢复。
完整计划后端的 request_hold_all 明确返回 unavailable / all_stopped=false，不发送未经验证的候选停止动作；因此该后端仍 blocked。监督单次不改变此结果。
submit 为同步阻塞调用，当前串行 MCP 进程不能同时处理取消；需另一个进程请求取消，或 Ctrl-C。另一进程可直接用已知 preview_id 查询或取消，无需另读文件查执行编号。
取消响应、CAN send 返回和断开连接均不证明已停。进程被终止或连接中断后状态可能未知；只能读取新反馈后另行处理，不自动重放或续跑。
发送统计区分尝试与发送返回；统计成功不代表控制器接受，更不代表物理完成。

## 传输与权限

使用无第三方 MCP 依赖的 stdio JSON-RPC 小实现，协议固定 2025-06-18。Python/native SDK stdout 被转到 stderr，stdout 仅返回协议。
CLI `--call` 返回非零退出码表示工具拒绝/失败或调用错误，不能把 JSON 中的 blocked 当成命令执行成功。
支持 initialize、initialized 通知、ping、tools/list、tools/call；不支持资源、提示词分发、HTTP 或全功能协议。
依据：[MCP 生命周期](https://modelcontextprotocol.io/specification/2025-06-18/basic/lifecycle)、[stdio 传输](https://modelcontextprotocol.io/specification/2025-06-18/basic/transports)、[工具规范](https://modelcontextprotocol.io/specification/2025-06-18/server/tools)。
`configs/mcp.example.json` 是常见 mcpServers 形式示例；各客户端配置格式可能不同，尚未修改当前客户端设置。
本工具没有鉴权或网络服务，不能据此开放远程控制端口；本阶段只作为本地子进程使用。
MCP 不解除操作系统或会话的设备限制。本轮没有新增权限、代理、后台服务或自动启动项。
