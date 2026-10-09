# 接触与微调

用于当前已授权的抓取、固定、拔出或插入阶段。用户希望直接尝试接触并根据实际响应微调时，执行一次可被当前宿主准入的接触动作，再判断结果；不把“还没有发生接触”当作缺少能力或缺少授权。

## 三类事实分开

- **执行能力**：宿主能否执行本阶段的有界接触命令、读取结果并按现有合同处理异常。已有有效入口直接复用，不为每次接触重新审计、重做资格或请示。
- **动作前场景**：当前 RGB 可判断接触面/轴向、手指位置、原支撑及运动通道；双臂有同场景的独立反馈。沿用本任务已确认的断电与净空，不编造距离或旧坐标。
- **动作后结果**：是否接触、双侧是否夹住、插排是否保持原支撑、插头是否产生相对位移。结果通过实际动作后的新图与反馈建立，不能成为第一次建立该结果的循环前提。

局部遮挡本身不等同接触异常。视觉未知与执行回执未知分别记录；不能因为夹点看不全就自动认定滑移、支撑移动或卡滞。结合当前可见通道、已确认方向、夹爪/关节反馈及完整回执，足以支持一个已有入口内的有限动作时直接推进，按 [遮挡与继续动作](../../piper-task-pipeline/references/online-readiness.md#遮挡与继续动作) 处理。物体结果仍可保留未知；不把未知填写为双臂带载确认接口要求的 `progress` 或 `retained_local`。

## 每轮做一个接触目标

1. 从当前图像选一个明确目标：减小可见间隙、建立轻触、微调夹点，或沿已确认的轴向推进一段。使用现有阶段/宿主的速度、位移、姿态与夹爪力度上限；微调比当前段更小，不新增通用毫米数或放宽阈值。
2. 一次只动一臂，另一臂沿用当前独立静止回执。初次左臂接触插排时，右臂只需满足其静止侧合同；不要求左臂在自己闭爪前已夹住插排。进入右臂拔出前，才要求左臂固定插排的实际证据和适用的带载支撑能力。
3. 同一事件只派发一次，然后获取动作后的新 RGB、夹爪和双臂反馈。物体仍受原支撑、反馈正常而接触点略偏时，在下一周期提出一个有方向依据的更小修正；不因尚未完成整项接触而退回 startup/prepare。
4. 回执已明确结束且宿主仍准入，才能根据新场景发下一事件。通信失效、未知/部分发送或故障锁存时，微调也属于新运动，不能用它补发上一目标或绕过拒绝。有证据表明卡滞、滑移、支撑移动，或达到原无进展预算时停止新增目标，按现有适用异常处置处理，不加力硬推；单纯遮挡不作为这些异常的证据。
5. 接触尝试和微调计入原阶段 `max_cycles`、无进展与总预算；下一轮目标对应当前图像和反馈支持的局部修正。未知结果不能冒充物体进展，也不因遮挡增加或清零预算，不通过换阶段名、run-id 或观察分支清零。

在座插头先建立支持下的抓持，再进入明确拔出阶段；不能用试提验证取代拔出。插入每一段依据插头与目标孔的实际对齐，机器人到位不等于插头已插入。完成仍按冻结的拔出、左孔就位、双侧释放与稳定证据判断。

## 夹爪接触回执

夹爪接触物体后，实际开口可能大于目标开口；这本身既不能证明夹住，也不应在设计接触流程时一律等同通信或电机故障。应由支持该模式的宿主明确区分正常接触结束、目标到位、继续运动及异常，再结合新图判断是否需要微调。不能由模型擅自将未到位、超时或未知发送改写成接触成功。

当前 `GuardedPairDevice` 已有 `gripper_contact_observation=true`。在健康、已使能、持续连接的 pair 宿主内，以 `robot_pair_submit_once` 的 `operation=grip_supported`、`kind=gripper` 发一次闭合目标；相对新反馈的闭合不超过代码中的 5 mm，复用原名义力 0.2 及其余反馈边界。请求、编码后的目标和实际发送前反馈均须符合该范围。没有接触成功证据不会阻止首次闭合。

先说明计划幅度，再获取并审阅本段新图；完成视觉判断后紧接着调用 observe 和已选定的单次 submit，避免在图像窗口内穿插诊断或长篇解释。接触闭合与支持式开爪也在 claim 前复用现有 RGB 执行余量检查：不足时返回 `refresh_required`，不建 episode、不占步数、不发送也不锁存；刷新图像后重新判断。已 claim 的故障仍不得自动重发。2026-10-08 的 `left-supported-reacquire-1` 因提交时图龄约 27 秒、余额不足三秒基线而在零 TX 时失败；[实际结果](../../../../artifacts/supported_contact_reacquisition_20261008/actual_result.json)保留，不能记为已试夹或用本修复改写成功。

回执 `execution_mode=contact_probe` 只区分 `target_arrived`、`settled_contact_candidate`、`unconfirmed`。该闭合响应候选要求可辨认的实际闭合和三秒完整稳定反馈；无响应或仅有力度读数不能成为候选。`accepted`、物理停止和物体接触确认仍未知，`grasp_verified=false`。先看动作后的新 RGB，再判断物体是否仍受原支撑；不能从候选推进到拔出或带载动作。

候选的旧闭爪目标可能仍有效，宿主将它记为 `unresolved_gripper_probe`。需要接续另一臂时，首次 probe 带上 `grasp_object_id`，让宿主在发送前建立按臂 episode；之后读取新 RGB，物体确实处于手指间且原支撑仍在时，调用 `robot_pair_retain_grasp` 记录视觉描述和当前 episode/observation。适配器自行采新三秒双臂反馈，零 TX 保留原目标、名义力和锚点；调用者不能传入硬件 measurement/contract 或 verified 布尔值。升级后 `retained_static` 允许另一空臂接近、对齐、支持下试夹，或满足下述确认条件后的空爪撤离；两臂身份和保持状态独立，不能据此拔出或搬运持夹物。

有身份的候选或静态保持可按新场景显式提交同爪 `operation=release_retreat`、`kind=gripper`。每次需 `release_support_observation` 描述当前图中的独立支撑，且 `release_support_relation="independent_support_present"`；不要求开爪前已与夹指分离。开度增量仍最多 5 mm，实际开口增加、到位及稳定只记 `release_opened`，该侧继续未决，另一侧记录不变。需要再开大时取新图后再次提交一个有界目标，原关节/位姿锚与总预算不变，不重做 prepare。

最后开爪后，取新保存 RGB 并调用 `robot_pair_confirm_release`，传 event/episode/observation-id、`visual_description`、`object_relation="object_clear_of_fingers"`、`support_relation="independent_support_present"`。适配器取得新的三秒稳定 trace，宿主匹配最后 opening 的事件、目标、时间和摘要；持久确认与新反馈核对后才零 TX 清理本地未决状态、记 `released`。支持物可以是本任务左侧目标插孔，不要求仍在原孔；这也不证明插对目标或最终稳定。模型语义、图像来源和机器测量分开记录。

确认后的空爪撤离使用 `kind=joint, operation=release_retreat`，每段提供新 RGB 的 `release_retreat_observation`，描述夹爪已空且与物体分离。宿主还检查当前确认 token、另一侧回执及原关节/几何/预算；新夹爪 claim 使该侧 token 失效。不要把撤离改名接近绕过，也不自动开爪、撤离或重发。未完成分离确认就 close/EOF 仍锁存；旧 v1 机械 `released` 仅作历史，不能自动继承新语义。未绑定 episode 的旧 probe 只做机械残余清理，不记录物体释放。

此释放流程覆盖候选/静态保持及已确认新图响应的 retained_local；后者以真实完成回执的当前局部关节锚核对，原始抓持锚保留。旧候选、失败和迟到回执保留真实时间与发送事实，不给下一步续资格。实物目标关系、双手撤离后的稳定及任务成功仍需独立新图证据，不能由 `released` 或机器人到位代替。

## 已开爪后在原支撑下重新试夹

用户要求左臂直接试夹、右臂不动作时，已完整成功的支持式开爪可经[原预算审计接续](../../piper-task-pipeline/references/audited-restart.md#已成功开爪后的支持下重新试夹)进入有限 `reacquisition` 范围。当前图像仍显示原独立支撑、手指与物体可能接触时，可以重新轻夹；不要求先证明 `object_clear_of_fingers`，也不把开爪到位记作空爪或已释放。右指尖是否仍接触按新图如实记录，本范围内右臂保持零 TX。

沿用 `robot_pair_submit_once(arm="left", kind="gripper", operation="grip_supported")`，提供当前 `grasp_object_id`、新图的 `probe_support_observation` 和 `probe_support_relation="independent_support_present"`。宿主自行绑定已审计来源；每次闭合的请求和编码目标相对实际开口均不超过 5 mm，名义力仍为 0.2，发送前保留三秒本体与爪基线及原反馈限制。两臂本体始终沿用原 candidate 来源参考；首次爪参考用最近成功开爪的实际宽度，后续只用上一段最终实测宽度，不重置本体参考。

同一审计范围最多三次试夹，每次先读新 RGB。只有前次完整成功且结果为 `target_arrived`、没有接触候选时，才能提出比前次请求及编码目标都更窄的新闭合。`settled_contact_candidate` 后停止新增闭合，用动作后的新图和 `robot_pair_retain_grasp` 做零 TX 保持核验；失败、无响应或未知发送不重试。候选、保持和三次额度都不授予本体运动、右臂动作或单左臂拔出资格，也不补建关节缓存。

恢复试夹中的身体参考须在单动作、持久静止检查和原接触检查间保持一致；新的发送前基线用于三秒稳定性和夹爪检查，不另外移动身体允许范围。2026-10-08 的 `left-contact-round-probe-1` 已发送一帧，左爪实际从 38.36 mm 收至后来观测的 33.88 mm，却先被新身体基线拒绝；原身体参考下，动作及故障后的全部记录均在原范围内。见[实际回执](../../../../artifacts/contact_zero_tx_restart_20261008/actual_action_result.json)。这个问题应修正参考选择，不能提高阈值或追认旧步成功；已发送后的实际爪宽也不能冒充先前成功开爪的宽度。后续只能经适用的审计接续保留原失败、实际发送和剩余额度，结合新图与新反馈选择下一目标。

已完整发送闭爪目标、但开始时已接触而闭合响应不足的情况，另用 `robot_pair_observe_supported_contact` 的零 TX 当前接触证据。须先经 `prepare_existing_contact_observation` / `activate_existing_contact_observation` 审计原失败、旧宿主关闭、现场双侧接触来源及新图和被动反馈；不能在锁存宿主直接调用。工具绑定新场景、物体身份和独立支撑描述，设备重新采完整三秒反馈，记录相对现存目标持续未到位的开口、原身体参考和新爪宽。原闭合判据、失败和三次已耗额度不改；现场回答与 RGB 来源分别记，力读数仅作辅助。新观察占一次总步数，成功可再用新图接共用零 TX 静态保持，仍不授予本体运动、加力或带载资格。

## 图像监督拔出、搬移与插入

本任务的 `kind="joint"`、`admission_mode="rgb_supervised"` 带载分支按冻结的 `worker_arm`／`support_arm` 执行 `extract_segment`、`transport`、`insert_segment`。旧任务未指定时保持右工作、左支撑；新任务可指定左工作、右支撑。提交当前 `loaded_observation`、`corridor_observation`、冻结的 `source_object_id`、`target_object_id`，宿主自行绑定两臂的真实抓持 episode、版本、夹爪目标和反馈。支撑臂保持桌面插排，不能当观察臂；不要求在第一次拔出前已经拔出成功。前文左扶右拔为默认分工示例，不限制新任务的角色选择。

拔出/插入请求及编码后的模型末端目标相对当前/原点均最多 2 mm、0.01 rad；这是保守软件分段策略，不是厂家精度或力控。搬移沿用原关节步长与位姿上限；接近左目标时缩小搬移步长以对齐实际插脚。发送后仍逐样本检查硬边界、反馈、支撑臂原锚和双夹爪；只动工作臂本体，原始抓持锚不重写。完整四帧及稳定到位才记录当前局部锚，随后进入 `loaded_pending_visual`，阻止下一目标和开爪。

读取每段后的新图，调用 `robot_pair_confirm_loaded_response`，绑定刚完成的 `action_event_id`。`object_relation` 描述插头仍在工作臂夹指间；`support_relation="table_supported_stationary"` 描述支撑臂所扶的桌面插排，不是悬空插头。`task_relation` 按实际新图记录原孔仍接合、已分离、左孔对齐、部分插入或就位。未知、滑移、支撑移动锁存；累计两次 `no_progress` 禁止继续分段，换操作不清零。无进展不能同时声称已进入下一物体关系。新图语义单独记录，不改名为力或独立视觉认证。

确认 `retained_local` 后才能下一段或按当前独立支撑开爪；最后仍经 `robot_pair_confirm_release` 确认分离。空爪撤离可使用显式 RGB `joint release_retreat`，保留该侧当前 release token、新空爪描述和整臂通道，不改名为空爪接近绕过。`contact_step_supported`、`contact_support_verified` 全局值仍不表示通用力控资格；上述专用事件合同不是任意带载动作的许可。取消/故障停止新增发送，物理停止未知仍为 null，不自动重发或加力。

恢复开爪已完整发送、却因原抓取参考偏差失败时，按[原预算内的开爪接续](../../piper-task-pipeline/references/audited-restart.md#已完整开爪后原参考偏差的接续)处理。保留旧本体参考，将后来已改变的爪宽单独作为残余开口证据；不能用旧爪宽阻止适用的新开爪，也不能据此重置身体参考或直接认定已经分离。

轻触与微调沿用上述固定工具和现有操作名，不引入自由控制脚本；未覆盖的动作不因技能文字获得执行资格。
