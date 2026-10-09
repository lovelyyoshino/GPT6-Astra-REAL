---
name: piper-arm-coordination
description: 为 Piper 双臂任务选择持久有界宿主或离线协同契约，核对共同场景、独立保持回执、一次派发和跨进程故障；区分静止观察与接触支撑资格。
---

# 协同分支

任务涉及第二条臂时才加载本入口；普通单臂只走原 pipeline。在 `projects/piper_right_pick_demo` 下查看目标模式：

```bash
PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --task cups --mode dual_arm
PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --task pen --mode worker_with_observer --worker-arm left --compact
```

双臂任务使用 worker_arm 和 peer_arm，两侧均为任务角色；观察模式使用 worker_arm 和 observer_arm，第二侧只能 view_only。双臂拔帽、旋瓶盖和旋螺母需要固定物体，不能伪装为辅助观察。

共享一个 coordinator、场景版本和总预算；本批 ARX5 recipe 一次仅动一臂。另一侧的 null/held 要有同一场景下的独立静止保持回执。左右提案、整臂通道、共同 duration、双方资格全部通过后才允许发送。

需要换视角时：验证最新主臂 hold → 观察臂有限换位 → 独立回执 → 新双臂观测 → 恢复原主臂 stage。最多两次观察换位，消耗同一总预算；观察臂不能抓取、托举或固定目标。

一侧缺回执或部分发送，锁存整对；不补发另一侧、不重启控制宿主清故障。共享 barrier 不代表固件原子提交或硬同步。

用户要求实机双任务臂且现有条件适用时，使用 [持久双臂宿主](../../../projects/piperx_cloth_demo/docs/PAIR_HOST.md)：长期 stdio server 内的 `robot_pair_open → robot_pair_observe → robot_pair_submit_once → robot_pair_status`。宿主保持同一双臂连接、一次只发一臂，生成绑定当前 owner/scene 的 peer 回执；由宿主监测反馈，不接受调用者传入 `hold_verified=true` 代替。`--call` 单次进程不能开启该会话。相同 event-id 只读旧结果，未决发送或故障禁止换 run-id 重试。

建立接触与利用已建立的支撑分阶段处理，按 [接触与微调](../piper-manipulation/references/bounded-contact.md) 用一次低速接触和新反馈逐步建立证据。不要要求左臂第一次接触插排之前已固定插排；左臂建立抓持时右臂是静止侧，右臂真正拔出时才需要左臂的当前固定证据和适用带载支撑。用户已授权这类任务就直接推进宿主允许的有界步骤，不另问是否允许轻触。

当前真实适配器已实现 `retained_target_stationary`、`gripper_contact_observation` 和 `gripper_static_retention`：首次有界闭爪可返回接触候选；带 `grasp_object_id` 的候选在同库按臂登记。动作后新 RGB 与适配器新三秒 trace 经 `robot_pair_retain_grasp` 零 TX 保留原目标，另一空臂可接近、对齐或试夹。两臂的 episode/原始锚点/释放分别保存；候选和静态保持都不能当作左臂已能带载固定。通用 `contact_step_supported`、`contact_support_verified` 不被提升为真；拔插工作臂已有独立的 RGB 带载事件合同，按 [接触分支](../piper-manipulation/references/bounded-contact.md) 绑定双臂原抓持记录、当前局部锚与每段新图响应。此分支仍不提供物理停止或插拔力认证。MOVE_L/夹爪路径的 `robot_pair_cancel` 只阻止新增软件发送；未知停止继续记为 null。客户端意外退出或带未解除候选关闭会持久锁存，不静默断开后重派。

`tasks/plug_transfer_left.json` 保留默认左臂固定、右臂拔插；新任务以 `worker_arm`／`support_arm` 显式冻结反向分工时，通过 `plug_recipe.render_plug_recipe` 派生同一任务。左侧目标插孔身份和双侧释放后的新图证据不变，源 recipe 与派生文件分别绑定哈希，不自带实机资格。旧 `execution.py` paired 仍为 near-time 发送，通用 fast CLI/`./astra` 协同仍离线。适配范围与剩余缺口见 [协同审查](../../../projects/piper_right_pick_demo/docs/ATOMIC_SKILLS_AND_DUAL_COORDINATION.md)。

PiPER X 的新增 `kind=joint` 路径将官方模型几何与控制器原始位姿分开，沿用单次派发和账本。同连接准备、十二关节限位查询、来源读取及首次目标初始化已接入生产工具，按 [在线接续分流](../piper-task-pipeline/references/online-readiness.md) 复用同 owner 和预算，不启用独立旧发送入口。`robot_pair_initialize_joint_target` 接受当前 P/J/L 起点，按独立监督合同建立合法非零目标或完成适用 J2/J3 边界恢复；完整四帧、新 J 反馈及三秒稳定到位后才生成真实缓存，另一臂和夹爪原锚点保留。未知旧缓存激活与部分更新风险仍明确记录；普通关节路径的已知缓存、0.05 rad 总旋转及 hold 条件不是该初始化的前提。

边界向内接续已通过同一 `kind=joint` 的 `approach`/`align` 接通。宿主核对 live 初始化来源与 exact cache、owner/连接及有效限位，限定 J2 下界/J3 上界的 0.003 rad 反馈尾差；目标内侧 0.010 rad、单轴步长 0.025 rad 不变。新的 X 模型完整独立轴包络保持 20 mm/0.05 rad、原 hold 预留和整臂净空条件，raw 反馈独立监测。无需新工具、参数或调用者资格布尔值，具体分流见上述在线接续说明。

同模式保持仍只针对满足原 hold 合同的普通 MOVE_J，在完整原发送返回后由明确客户端取消申请一次。若本次动作原点任一臂仍有名义越界尾差，计划会报告 `hold_reference_within_nominal_limits=false`：现有 hold 构造入口要求双臂原点严格名义合法，该动作的尾差接纳不授予保持例外。首次初始化、未知/部分发送、EOF、一般故障及后续独立故障也不授予该例外；未知物理停止保留 null，保持观测不等于物理停止。

普通空载关节接近/对齐可在同一宿主显式选择 `rgb_supervised`，以当前工作臂空载及整臂通道语义接续；另一臂原有静态夹持状态与独立回执仍保留，不被重标为观察臂或带载支撑。此分支为 `latch_only`，无米制 hold 派发；只有完整四帧后允许有界短暂收敛，最终仍稳定到位。具体字段、幅度和期限见 [在线接续分流](../piper-task-pipeline/references/online-readiness.md)。米制分支现场几何、专用带载动作、释放/撤离与 RGB 物体进展仍按各自入口分别满足。官方 SDK/FakeCAN 贯穿测试不提供现场资格，不能把测试中的合成几何或纯计划当作当前证据。实现顺序与完整插拔设计见 [PiPER X 路径设计](../../../projects/piperx_cloth_demo/docs/PIPER_X_JOINT_PATH_DESIGN.md)。
