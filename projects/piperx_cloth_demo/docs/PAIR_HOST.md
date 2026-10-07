# 持久双臂有界宿主

`pair_host.py` 持有同一组两臂 SDK 连接，`pair_device.py` 复用现成单臂的白名单发送、反馈窗口和漂移边界。每次只操作一臂；另一臂继续读取反馈且禁止 TX。`pair_ledger.py` 在 SQLite 提交发送 claim 后才允许唯一一次尝试，原有通用 full-plan/fast 协同入口不因此解锁。

构造对象、查询 schema 和离线测试不连接设备。明确调用 `robot_pair_open` 才建立连接：默认 `connection_mode=ready` 要求已经使能、健康、CAN 模式的双臂与夹爪；`prepare` 分支零 TX 保留当前已知模式/使能状态并观察。两个分支都不自动 startup、恢复边界或开合夹爪。源码更新与在线执行分开进行。

## 固定接口与生命周期

在本项目下运行长期 MCP stdio server：

```bash
/home/agilex/miniconda3/envs/pi0_infer/bin/python3.10 -B -m robot_tools.server
```

这是 MCP JSON-RPC 服务，不是接收任意 Python 的 shell。`--call robot_pair_open` 会在设备访问前拒绝，避免创建立即退出的控制宿主。普通已有单次工具仍保留。

| 工具 | 用途 |
| --- | --- |
| `robot_pair_open` | 冻结 run-id、task-id、双任务臂角色、当前用户净空陈述及动作/时间预算，独占连接并采集三秒新鲜基线。默认最多 128 次派发尝试、900 秒；不重置旧运行。 |
| `robot_pair_observe` | 读取现成录像 worker 保存的 `observation.json`，核对三路 serial、递增 frame、图龄与 skew；同宿主读取新机器人反馈，生成共同场景和不可替代的 peer receipt-id。不会启动相机。 |
| `robot_pair_publish_geometry` | 用当前 observation-id 和受控资料集 record-set-id 导入真实测量／安装记录；自动绑定 owner、连接、图像及哈希，零 TX。缺记录或过期不会生成默认几何。 |
| `robot_pair_submit_once` | 指定 event-id、当前 observation-id、另一臂 receipt-id、arm、kind、operation 及唯一目标。支持 `move` / `joint` / `gripper` 的软件分流；joint 需已完成的同连接目标历史及本段来源；普通 approach/align 可显式选择当前 RGB 监督，默认米制入口仍需实际几何。来源绑定的初始化后边界内移已接通。持久 claim 后异步执行一次，不再开启设备连接。 |
| `robot_pair_initialize_joint_target` | 当前三路 RGB 的空载语义及内部来源齐备后，在原连接执行一次合法当前位置或 J2/J3 最近边界目标；准备态可调用。完整四帧、新 J 模式反馈和三秒稳定后才建立首次缓存；已有缓存零 TX 返回原来源。 |
| `robot_pair_retain_grasp` | 对已登记物体身份的夹爪候选采集新三秒反馈，保留原目标、力度和锚点，零 TX 建立静态保持；绑定当前 RGB 与单列的模型视觉陈述。 |
| `robot_pair_confirm_release` | 对最后一次开爪之后的新保存 RGB 记录物体与夹指分离、独立支撑语义；适配器取得新三秒稳定反馈，持久确认后零 TX 清除该侧本地未决记录。不是目标孔正确、最终稳定或物理停止证明。 |
| `robot_pair_status` | 查询宿主或 event 回执；动作运行时仍可响应。接收/到位/静止/物体成功/停止分别记录。 |
| `robot_pair_cancel` | 立即阻止普通新增帧并落盘。仅明确取消当前完整已返、同存活 worker/连接的 MOVE_J，可进入一次单独预算的同模式 hold；部分/未知发送、普通故障及 EOF 不允许该分支。已发帧保留，不发送 reset、disable 或猜测的 stop；不宣称已物理停止。 |
| `robot_pair_close` | 无未决动作时关闭资源。带未解除接触候选关闭会持久锁存；正常关闭后可用新 owner 接续同 run 的剩余预算，有故障不能重开。断连不证明物理停止。 |

move 仍是固定 1% MOVE_L，最大 30 mm/0.05 rad；jaw 是 0–55 mm、厂家名义力 0.2 的一次目标。`target_pose_m_rad`、`target_joints_rad`、`width_m` 必须且只能提供一个，并与 kind 匹配。普通位置路径保留到位要求。`grip_supported` + `kind=gripper` 使用闭合增量不超过 5 mm 的接触观测路径，分别返回到位、稳定接触候选或未确认；候选不是到位或抓牢。

新增 `joint` 为六轴明确目标的独立软件合同，支持空载 `approach` / `align`，以及下述确认释放后的空爪 `release_retreat`。各轴请求和编码后步长均 ≤0.025 rad，目标距有效限位 ≥0.010 rad；联合几何及保持预算可要求更小幅度。已固定官方 X 模型，SDK/URDF/控制器限位显式取交集；保留 raw controller pose 与模型 FK 两条来源，不覆盖原反馈，也不改变 MOVE_L 的一致性检查。模型端点 ≤15 mm，法兰动作/保持总幅度 ≤20 mm/0.05 rad，1% SDK 速度档、逐帧 50 ms 双臂反馈及前后三秒稳定窗口保持不变。1% 不是实测速度。

`PairHost._joint_context` 只接受内部 `joint_sources_provider(scene, arm)` 解析的模型、URDF、控制器限位及当前几何来源；工具调用不能上传一份资格字典来替代它。正常服务已安装 `JointSourcesProvider`，官方模型字节固定在 `data/piper_x_official`，读取不会自动查询设备或下载资料。当前连接的限位可通过下述一次查询写入来源索引；现场几何记录不会自动生成。设备首次连接的 `_joint_cache` 明确未知，由独立初始化建立，普通关节入口不从实测 q 伪造历史。相机、支架、电缆和净空须绑定实际来源，不用默认尺寸填空，也不反复向用户索取通用数值。普通路径的边界恢复候选仍未接入其运输层；独立 startup 合同与它分开。

同连接准备已通过 `PairHost/ToolService` 接通：`robot_pair_open(connection_mode="prepare")` 零 TX 建立静止观察；用 `robot_pair_observe` 取得当前保存的三路 RGB 场景后，调用 `robot_pair_prepare_gripper(event_id, observation_id, arm, empty_jaw_observation)`。描述明确记录为模型语义，不冒充空爪传感器证明。选中臂需已 CAN 控制且六关节使能；夹爪未使能时至多发一个实测开口 0x159，已使能时不发帧。请求先占用原账本一个事件，再在原连接异步执行；同事件重放只读原回执。动作后须取新图，不能给下一爪沿用旧场景。

`robot_pair_promote_ready()` 零 TX 核对原任务条件，整个升级窗口继续检查原姿态/模式/使能/已准备夹爪目标，不重新锚定未命令的关节或位姿。已知缺少准备条件返回 `preparation_required`；真实反馈异常或发送不确定仍锁存。准备态可读取、查询及显式初始化；普通目标和抓持接续在派发 claim 前拒绝且不因尚未就绪而锁存。`task_ready=true` 只表示原任务反馈条件已满足，不产生首目标历史、几何、带载接触或停止资格。同连接关节使能和独立 CAN 接管仍缺，不能通过独立旧入口绕过已存在的 owner。

`robot_pair_initialize_joint_target(event_id, observation_id, arm, unloaded_observation)` 将当前场景、实际设备身份和原始反馈绑定到内部官方模型/限位/几何来源，再在原账本领取一次初始化事件。目标自动选择合法当前位置或最近 J2/J3 边界，不接受任意目标、context 或缓存。它复用既有独立监督 startup 范围，允许 P/J/L 已知模式和已知失能空爪；未知旧目标激活及四帧非原子风险明示，不要求先有普通动作的已知缓存或 hold。完成后仅更新选中臂已命令的姿态/模式锚，另一臂及夹爪原锚不变。SDK IntEnum 在 claim 前冻结为普通 JSON 整数，账本和 worker 使用同一份值。部分发送、模式回退、超时或来源变化锁存，禁止补帧。已有缓存时只读新反馈并返回原缓存；同 event-id 重放不再消耗事件，新 event-id 的零 TX 检查仍计入原预算。合法内侧非零起点可接普通小步；边界尾差由下述来源绑定接续处理，不通过反复初始化消除。

显式采用图像监督初始化时，给同一工具增加 `admission_mode="rgb_supervised"` 和 `corridor_observation`，具体描述本段双臂整链、夹爪/相机/支架、线缆和桌面的可见运动通道；`unloaded_observation` 仍描述双臂空爪、无接触。沿用用户当前净空陈述，不能把遮挡或不明通道描述为已确认。宿主保存当前三图及哈希、设备/owner/动作身份和原图像期限，读取当前连接的限位；不生成附件半径、米制净空或绝对工作区数据。固定低速、关节/相对位移、反馈、原锚和一次发送限制不变，但此分支明确不提供米制清扫/绝对工作区证明。默认 `metric_geometry` 原合同不变，缺资料不会自动切换分支。本初始化入口只覆盖首次合法种子或最近边界目标，不授予后续普通运动、持物拔插或停止资格；后续阶段必须有自己的适用入口。

图像监督初始化将发送过程与发送后的收敛观察分开：四帧完整返回前仍按原 ±0.003 rad 跟踪带检查；完整发送后只接收反馈，允许起点至编码目标区间外最多 0.025 rad 的暂态，超出原跟踪带累计最多 1 秒。任一相邻样本端点越带就计入整个时间间隔；回带、换关节或重建稳定窗口均不清零累计时间。仍检查原点每轴 0.10 rad、原有效关节反馈边界、raw/FK 各 20 mm、另一臂/夹爪、新鲜度与原截止时间。这里的 0.025 rad 是软件收敛观察策略，不是厂家精度，不扩大目标或发送前范围；默认米制分支仍严格检查原带宽。最终仍须回到目标 ±0.003 rad 和原 FK 到位范围，再通过三秒/至少 20 次新反馈的稳定窗口，才建立缓存。回执记录暂态峰值、累计越带时间和首个失败样本；不因等待收敛而补发目标。

普通空载 `joint` 接近/对齐也已接入显式图像监督：在同一 `robot_pair_submit_once` 上使用 `admission_mode="rgb_supervised"`、`unloaded_observation` 和 `corridor_observation`。前者描述本次工作臂空载且无接触；另一臂可按已有合同静态持夹，不要求它也空载。后者描述本段双臂及附件的可见通道。宿主绑定当前三图、operation、编码目标和真实同连接缓存，从官方固定模型及当前限位读取数值；不生成全任务轨迹或米制现场数值。保持每轴目标 0.025 rad、目标内侧 0.010 rad、15 mm 模型终点、20 mm/0.05 rad 相对反馈及原数值余量；临近目标按新图选更小目标。该分支明确 `hold_policy="latch_only"`，取消/EOF/故障停止新增发送，不派发米制 hold，也不声称物理已停止；默认米制入口及其适用的 hold 合同保持独立。

普通图像监督动作也区分发送中与发送后：四帧完整返回前保留原跟踪带和包络检查；完整发送后才允许有限接收观察，原点至编码目标区间外各最多 0.025 rad，再交原点每轴 ±0.028 rad（原普通最大步长加跟踪带）。超原跟踪带累计最多 1 秒，回带、换轴和稳定窗口重建不清零；最终仍需目标 ±0.003 rad 与原三秒/20 次稳定反馈。新接收观察范围不冒称原发送包络证明，原有效关节界、raw/FK 各 20 mm/0.05 rad、另一臂、夹爪、新鲜度与截止时间逐样本检查。初始化的 0.10 rad 恢复范围不用于普通动作；完整发送历史也不等于物体任务成功。

初始化后的普通内移已接入原 `kind=joint`，无需新工具或调用者资格参数。宿主和设备分别核对当前 live 来源与完整缓存、run/owner/epoch/连接、全套有效限位及四帧完成证据；新反馈须晚于完成记录。仅初始化目标恰为 J2 下界/J3 上界时，允许该轴最多 0.003 rad 的对应观察尾差，另一臂也须有自己的有效来源；名义限位、目标内侧 0.010 rad、单轴 0.025 rad、原锚和整臂净空不变。固定 X 模型的独立区间上界改进法兰位置/旋转计算，仍保留原 hold 预算。普通目标换缓存后旧来源失效；MOVE_L 任意发送尝试使该侧 joint 缓存/来源失效，夹爪动作不清除关节来源。

`robot_pair_inspect_joint_limits(event_id)` 无需物体 RGB，在原连接逐臂逐关节至多发送十二个精确 0x472 查询，不发运动、模式、使能或配置帧。夹爪可以保持已知失能状态。每个回复由独立原始 0x473 窗口关联；SDK 共享缓存/时间戳不充当新回复。缺失、重复、错关节、非法或部分发送立即结束序列，不自动重试。完整结果由宿主绑定 owner/run/连接，写入内容寻址文件并原子更新 `joint_sources/epochs/<scope-sha256>/index.json`；发布失败保留故障及发送事实，不给模型上传来源的入口。该查询消耗原任务一次事件及实际时间，查询成功不是运动资格。

来源目录按 run、owner、profile 和两侧完整连接身份区分。同一连接沿用该目录已有来源；新 owner 或任一侧连接变化时单独保存新来源，保留旧目录原字节，不把历史限位或几何改绑为当前。旧版 `joint_sources/index.json` 仅在身份精确匹配时兼容读取，不迁移或覆盖；已有但缺损的新目录不能退回旧来源。首次向新目录发布后，未发布的另一类来源也不会从旧版目录补入，因此只读兼容不是透明升级；不能热修改运行代码并继续沿用旧宿主。此分目录处理不重置预算、不继承目标缓存，也不替代新连接实际采集或当前场景资料。

接触候选要求测到实际闭合、完整三秒稳定窗口和新鲜双臂反馈；无响应、异常或仅力度读数不会冒充接触。精确分类 trace 单独写入 journal 的 `bounded_probe_trace` 事件，回执只携带摘要/哈希，避免超过持久事件大小。写日志和分类后再读取新反馈，不刷新旧样本的时间。

候选使 `unresolved_gripper_probe` 保留旧闭合目标可能仍有效的事实，阻止两臂其他目标。已登记物体身份的候选按下述释放流程接续，机械开爪到位仍保留 `release_opened` 未决状态。未绑定 episode 的旧观测 probe 仅在有界开爪到位后清理本地机械残余，不记录物体已释放。没有自动开爪、取消或重试；关闭、EOF 和异常不证明物理停止。

客户端意外 EOF 时整对锁存，不触发专用 joint hold；有未决动作先阻止新增帧并等待有限时间保存结果。未能完成退出的动作保持不确定，不能把进程退出当成停止。显式 close 不发送回零或松爪。

故障后的派发禁令与读取反馈分离。宿主仍打开时，后台 poll 只要能非阻塞取得 device lock，就可通过 `observe_fault_feedback()` 复制原 SDK RX 缓存；即使 `active_event_id` 因记账尚未清除，也不一概跳过读取。实际执行/保持 worker 仍占用设备锁时则明确暂缓。不调用查询、连接、准备或发送接口。`status.fault_feedback` 保存原始两臂反馈及逐分片的 missing/invalid/repeated/advanced/regressed、数据年龄和异常信息。`fault_feedback_read_state` 区分尚未读取、设备忙暂缓和已关闭；缓存采集时间不被状态查询刷新。它不发放新 scene/peer receipt，也不认证静止或物理停止。故障后账本状态用 `peek_status()` 只读事务，时钟异常不会因反复查询追加故障或续期。

普通 move 回执新增 `motion_effect`，绑定该动作派发前和执行后的反馈，分别记录请求向量、实测向量、沿目标方向及横向分量。小目标落在 5 mm 到位容差内不证明已经移动；超带横移不能被正向分量掩盖。这里的分辨带复用静止观测策略，不是传感器精度标定；物体进展、接触响应及任务成功保留独立字段。

连接失败仍保留宿主对象，供 status 查看及 close 清理，不丢弃故障设备句柄。EOF 等待两秒后仍未完成的动作保留 pending；此时不声称资源清理完成。

## 保持、净空与接触能力

几何资料接入使用 `robot_pair_publish_geometry(observation_id, record_set_id)`，读取固定 `site_records/piper_geometry/<id>/manifest.json` 引用的安装、工作区和当前净空资料。入口不接数值边界、任意路径、owner 或 `verified`。附件由法兰坐标中的完整部件包围盒及误差推导上界，工作区取两臂基座范围的保守交集，净空取当前各类表面距离扣除误差后的下界。它导入已取得的实际资料，不提供测量能力，不认证记录的物理真伪；源码见 `site_geometry_records.py`。来源生成与消费均核对时效，取得新 RGB 不能给旧净空续期。

测量记录原本的 `valid_until_s` 必须严格晚于当前场景最早 RGB 接收时间加 30 秒；不足以覆盖完整窗口的资料明确拒绝，不能改写其期限。初始化和普通关节路径沿用逐帧 RGB deadline；明确取消后的关节 hold 另在 bridge 构造时冻结同一截止时间，并在领取 hold 和最终每帧前核对。过期、回退或修改期限会持续阻止后续 hold 帧，已发生发送的回执仍可保存；这不证明物理停止。发布回执中的 `measurement_valid_until_s` 只报告原期限，不提供续期能力。

资料发布沿用原任务时限，不领取物理派发事件，不改变就绪、目标缓存或抓持状态。当前账本尚无代码修订迁移入口：干净关闭也不暂停总墙钟期限；修改控制代码后不能用新 run-id 或改时间字段绕过原合同。历史缺口见 [接续记录](../../../artifacts/plug_live_validation_1791357436415587512/continuation_gaps.json)；上文来源分目录修复不解除已到期预算或缺实际现场资料的阻碍。

2026-10-07 本次 [实机基础验证](../../../artifacts/plug_live_validation_1791357436415587512/live_audit.json) 已在非零起点完成两次实测开口夹爪使能及十二次限位查询，累计 14 个 CAN 帧，未发关节／末端目标。首次关节初始化因当前几何来源缺失在 claim 前拒绝；录像三路各 4768 帧完整解码并覆盖实际发送至持久回执。此结果不验证关节运动、抓持或插拔。

现场用户对本任务可行范围净空的确认可记入 `workspace_clearance_statement` 并沿用，不重复索取已经确认的信息，也不换算成未经测量的距离。它不是自碰撞模型、旧缓存目标范围或停止资格的证明。非零启动恢复的独立数值合同保持不变。

宿主的 peer 回执绑定 owner、scene、递增序号和实测反馈摘要。派发前重新检查另一臂；执行中持续核对；没有收到命令的臂/夹爪继续使用跨步冻结锚点，不能每步重锚来掩盖累计漂移。旧图换时间、旧 owner 回执和调用者的 `hold_verified=true` 均不能替代新回执。

三路 RGB 必须采集于本 owner 启动完成、上一动作完成之后，同时满足原有图龄、递增帧与时差约束；换 owner 不能让旧图重新成为当前场景。元数据只读一次并对同一份字节计算摘要，避免录像更新期间摘要与实际使用图像不一致。每个 CAN 帧发送前，在宿主账本检查返回后重新读取两臂反馈并检查新鲜度，不能用磁盘访问前的样本放行。

当前真实适配器能力如下：

| 能力 | 当前实现 |
| --- | --- |
| `retained_target_stationary` | 持续连接下、完整新反馈与固定锚点内的静止观察 |
| `gripper_contact_observation` | 有界闭爪响应分类及显式同爪有界开爪；有身份的物体需另确认分离，非实物接触资格证明 |
| `gripper_static_retention` | 原闭爪目标不变、新三秒反馈与固定锚点的零 TX 静态接续；只允许另一空臂准备，不证明带载支撑 |
| 六轴 `joint` 软件路径 | 纯候选、宿主桥、同连接真实 SDK 发送适配、生产来源 provider、独立初始化及来源绑定边界内移已接入；实际现场几何仍需补齐 |
| 同模式 MOVE_J hold | 明确取消、原四帧完整返回时的一次有界目标覆盖与实测静止记录；软件已实现，实机适用性未验证，不适用于 MOVE_L 或带载停止 |
| `contact_support_verified` | false；静止不证明带负载固定物体 |
| `contact_step_supported` | false；真实接触保持/故障后停止尚未具备独立资格 |
| `physical_stop_verified` | 回执中 null；cancel/close 只处理软件派发与资源 |

`grip_supported` 的夹爪观测不授予 `extract_segment`、`insert_segment` 等带载接触操作资格，只有下述独立 RGB 带载分支才处理相应事件，不能通过请求中的布尔开关开启。四帧 MOVE_L 仍不是固件原子提交：模式帧可能激活旧缓存，部分目标可能与旧目标混合；监测到偏差也不证明已物理停止。 宿主将 SDK 回执中的 IntEnum 状态先冻结为有限 JSON 数值再写严格账本，避免合法厂商枚举在动作已发送后触发格式故障；非法值仍拒绝，原始诊断与发送事实保留。

抓持接续已通过 `host_grasp.py` 接入同一宿主。首次支持下试夹在 `robot_pair_submit_once` 增加 `grasp_object_id`（例如 `strip` 或 `plug`）；宿主在发送前创建空 episode，在物理回执入账后解析适配器的原始 probe/trace，状态可从 `robot_pair_status.grasp_episodes` 读取。历史候选只记录事实，不要求磁盘入账发生在原 trace 的 100 ms 内，也不会给旧样本换时间；保持升级和派发仍要求新反馈。

下一轮用 `robot_pair_observe` 取得动作后的三路 RGB，外层模型查看图像，再调用 `robot_pair_retain_grasp`，传 event/episode/observation-id、`visual_description`、`object_relation=between_fingers`、`support_relation=original_support_present`。图像原字节哈希由服务解析并在升级时核对、归档；模型陈述明确标为语义观察，不冒充独立审核或抓力测量。硬件 measurement/retention_contract 不接受调用者传入，由适配器新采三秒反馈、记录原 trace 和签发来源；静态保持不重发闭爪目标、不加力，沿用原始锚点与总期限。

`retained_static` 允许另一空臂执行 `approach`/`align`、支持下试夹或已确认释放后的有界撤离，持夹臂只能明确开爪退出。两臂各自保存候选和保持记录；释放一侧不清除另一侧。每次接续比较适配器和持久记录的完整身份、原 probe、目标、锚点和合同。故障或未解决的夹持状态不能干净交接；原总预算不重置。右侧带载有限段现经独立 RGB 事件分支接入；物理力控与带载停止/保持保证仍未实现。

### 受支撑释放与空爪撤离

当前入口覆盖 `contact_candidate` / `retained_static` 的独立支撑释放，不覆盖尚未实现的 loaded 状态。每次 `robot_pair_submit_once(kind="gripper", operation="release_retreat")` 都需当前保存 RGB 的 `release_support_observation` 和 `release_support_relation="independent_support_present"`，开爪前不要求已经分离。释放意图先落抓持账本，再 claim 一次物理动作；相对新测开口增加最多 5 mm，名义力 0.2 不变。实际增加超过 0.5 mm、2 mm 内到位及三秒稳定后只记 `release_opened`。若仍需开大，取新图后再提交同爪有界目标；原关节/位姿锚不变，夹爪以最后实测开口为漂移锚，旧静态保持期限不续期，原任务总期限继续生效。

最后开爪之后取新图，调用 `robot_pair_confirm_release(event_id, episode_id, observation_id, visual_description, object_relation="object_clear_of_fingers", support_relation="independent_support_present")`。适配器提供新的三秒完整反馈，宿主绑定最后 opening 的事件/目标/trace/时间，持久保存后经新反馈核对才零 TX 清理本地记录并返回 `released`。这是模型 RGB 语义与机器人测量的分别记录，不是独立视觉认证、插对目标孔或最终稳定证明。v2 保存 opening/confirmation；v1 的机械 `released` 仅历史可读，不自动允许新 episode 或干净交接。失败、迟到和未知事实保留在原回执/账本，不能恢复资格。

随后每段空爪撤离使用 `kind="joint", operation="release_retreat"`，提供本段新图的 `release_retreat_observation`，描述夹指已空且与物体分离。它还需同宿主当前确认 token、另一臂独立回执及原关节/几何/预算守卫；新夹爪动作的 claim 会使该侧 token 失效。不会自动撤离，也不把旧 owner 的确认用于新连接。离线实现与验证记录见 [本轮释放流程](../../../artifacts/plug_release_flow_1791359593200793893/)；其中关节撤离测试显式注入测试用历史目标缓存，实际初始化建立缓存的回归另测，不能合称本轮无注入缓存的完整释放链。新的 RGB 分支可接续有限带载段与空爪撤离；实物夹持、物体响应和目标终态仍须现场新图确认。

### 同模式保持与取消

`hold_transaction.py` 本身仍是纯事务检查器；已由 `host_joint.py`、`pair_joint_adapter.py` 接入同一宿主、SQLite 账本和原 SDK 连接。原 MOVE_J 四帧完整返回并落盘后，明确客户端取消才可由原存活 worker 预留一次独立 hold 事件；它消耗原任务剩余预算，不清故障。每一帧先记录 pending，再取得新双臂 RX 并检查，随后才进入原精确字节白名单；部分/未知帧不补发，重启不能恢复发送许可。SDK 重复模式帧过滤保持原设置，适配器仅在原包络内作有限反馈等待。

取消先设置软件阻断标志。若专用取消请求的持久发布尚未完成，桥最多等待 25 ms；超时成为不可复活的失败，迟到的数据库提交不能再次解锁 hold。这个等待不是持续 RX 或实时停止保证，随后仍须以新反馈和真实发送回执判断。

保持目标在 durable claim 中冻结，数据库后的新样本按现有 0.003 rad 观察带及剩余几何预算核对，不要求逐 millidegree 相等。模型空间 hold 分支用固定官方 X FK 检查几何与 workspace，原控制器位姿独立监测相对漂移；旧绝对 FK 一致性分支不变。目标替换尝试后旧缓存资格失效，原动作和 hold 回执分别保存。只有到位和完整三秒窗口才报告 `hold_observed`，`accepted`、`original_target_cancelled`、`physical_stop_verified` 仍未知；后续普通动作仍被锁存。

来源绑定的内移不改变 hold 的名义限位守卫。原始双臂参考任一侧含边界外尾差时，plan 的 `hold_reference_within_nominal_limits=false`，hold constructor 拒绝 `nominal_joint_limit`；即使当前选中臂已经到内侧，也不能重写原参考来放行。对应离线取消回归没有发送 hold 帧，原故障和停止未知保留。两侧内移后，后续普通动作以自己的新原点另行判断，字段为 true 也不证明保持适用或物理停止。

真实 SDK + FakeCAN 已覆盖这一条软件发送路径；独立初始化另有从未知缓存到完整发送记录的测试，此前另有不注入缓存的双臂初始化→夹爪准备→内移→普通小步集成，见 [独立审查](../../../artifacts/plug_ingress_1791356964575384710/independent_ingress_review.json)。这些测试均不能代替实机验证。实测关节 q、`motion_status=0` 和 leader 输入不是已接受的 MOVE_J 目标缓存；不能用测试中注入的历史记录启动生产设备。完整实现路线、来源 adapter 和插拔各阶段的验收/退出见 [PiPER X 关节路径设计](PIPER_X_JOINT_PATH_DESIGN.md)。

接触实现及模型操作按 [接触与微调技能](../../../.agents/skills/piper-manipulation/references/bounded-contact.md) 区分执行能力与物体结果：首次轻触/抓持用于建立接触，完成证据在动作后取得；不能要求物体在首次闭爪之前已经夹住。实现范围以本表为准。

`tasks/plug_transfer_left.json` 将左臂固定插排、右臂拔出→移向左孔→插入→右侧释放→左侧释放→双新样本稳定分为 14 阶段。该 recipe 仍为离线合同；接触资格、现场图像和物体结果不会由软件测试或 task-id 自动产生。

进一步实现按以下证据顺序推进：复用唯一宿主与录像 → 当前来源、独立初始化及边界向内接续 → 左侧静态夹持 → 右侧静态夹持 → 一次有限轴向接触探测 → 带载分段抽出 → 完全脱离后持物横移、对齐及分段插入 → 右先释放、左后释放 → 两次相隔至少两秒的新图与反馈验证独立稳定。首次负载 probe 的前提是双侧静态夹持和原物体支撑；负载响应证据在这个动作后建立，不能要求动作前已有 loaded 证明，也不能把静态合同直接升级为带载资格。现在首段直接使用独立的 RGB `extract_segment`，后续按新图响应接 `transport` 与 `insert_segment`；不要求先有成功拔出轨迹。异常或未确认时停止新增段，不自动增幅/加力；没有适用停止回执时保留停止未知。

每段物体进展须由动作后新 RGB 及实测反馈支持。现有名义夹爪 force 和快照可选 `motor_feedback` 的六路电流/估算力矩只保留原值、独立时间和新鲜度，未接成带载异常处置合同，不虚构六维力传感器或电流到轴向牛顿的标定。缺少该可选诊断不改变原必需反馈或健康判据。右侧释放后须先确认新插孔独立支撑插头，才能释放左侧；任何后续动作或人工介入都会使旧终态证据失效。正常关闭录像、机械臂到位与插拔成功分别记录。

在仓库根目录用 `./astra plan --recipe tasks/plug_transfer_left.json --mode dual_arm` 查看合同，`replay` 使用相同参数生成明确标注的合成回放。单臂或观察臂模式会拒绝这项需要两只任务臂的任务。

## 持久性和适用范围

`runs/pair_sessions.sqlite` 保存冻结合同/代码摘要、owner、累计预算、事件和首个全局故障。同 event-id/同请求返回原回执；不同请求拒绝；pending 从不重发。claim 后崩溃即使没有证明实际发帧，也必须保留不确定。换 run-id 不会绕过本数据库故障；没有清 latch API。

同项目旧发送工具同时检查 pair owner、pending 和 fault，并复用 `execution.lock`；未决进程崩溃不能借旧入口绕过。该锁与账本不能排斥另一份项目目录或不合作的 ROS/SDK 发送方，仍按在线接续流程确认唯一发送方。复制目录、删除数据库或修改资格布尔值都不构成恢复流程。

新 SDK 适配器、宿主、账本和 recipe 均有禁止真实 socket 的测试，包括真实厂商编码器配合 FakeCAN。测试证明软件边界与故障处理，不证明实际接触支撑、物理停止或拔插成功。


## RGB 监督插拔有限段

`robot_pair_submit_once(kind="joint", arm="right", admission_mode="rgb_supervised")` 现在可显式选择 `extract_segment`、`transport`、`insert_segment`。同时传本次 `loaded_observation`、`corridor_observation` 与初次冻结的 `source_object_id`、`target_object_id`；不传空载描述。宿主内部绑定两侧当前 episode、真实 probe、owner/连接、已发送缓存和新三图，调用者不能注入这些机器证据。左侧必须是当前 `retained_static`，保持桌面支撑下的原关节/位姿锚和夹爪目标；这只证明所观测静态，没有验证抗拔力。

首次右侧 `retained_static` 即可提出一次有限抽出，不要求此前已拔出。`extract_segment` 和 `insert_segment` 的请求与量化目标相对当前/原始模型位姿均限制在 2 mm、0.01 rad；这是部署的软件目标上限，不是厂家精度或接触力上限。搬移仍用原普通关节限值。1% 指令、六轴步长、真实缓存与混合目标包络、原数值反馈界、双爪原目标、peer 静止、50 ms、RGB/总期限以及一次发送全部保留。RGB 取消仍为 latch-only，未知物理停止保持 null。

每段四帧返回并按原到位与三秒稳定要求结束后，状态仅为 `loaded_pending_visual`，禁止再发运动或开爪。调用 `robot_pair_confirm_loaded_response`，引用该动作和动作后新 `observation_id`，描述插头是否仍在右指间、**左臂所扶插排**是否仍受桌面支撑且未移动、插头与冻结源孔/目标孔的关系。传 `response=progress/no_progress/unknown/adverse`；支持关系 `table_supported_stationary` 指插排，不指已拔起的插头。机器测量由宿主新读，语义图像报告分别保存。确认是零 TX；未知、滑落、插排移动或不一致锁存故障，两次累计无进展后不再准入新段，不重置预算或增加力度。

原 `original_anchor`、probe 与所有段历史不变。只有完成回执建立的 `local_anchor` 才用于右侧段间静止监测；新图确认后记 `retained_local`。完全脱离源孔的响应才允许横移；带载对齐用 `transport` 的最后一小段并用新图记录 `target_aligned`，不能改名为空爪 `align`。当前目标对齐响应才允许插入。`contact_support_verified` 和通用 `contact_step_supported` 不被设真，电流/估算力矩只作未标定诊断。机器人到位、语义进展与整项任务成功分别记录。

右侧就位后复用受支撑 `release_retreat` 开爪和 `robot_pair_confirm_release`。该测量以真实最后局部锚为身体基准，仍保留初始 probe 锚。确认后空爪 `joint release_retreat` 也可显式 RGB 准入，用本段 `release_retreat_observation` 和 `corridor_observation`；仍先检查宿主已确认释放 token，不能改名普通接近绕过。右侧退出后再释放/撤离左侧。最终需至少相隔两秒的两次新图确认独立稳定；软件贯穿测试和发送完成都不替代该实物结果。
