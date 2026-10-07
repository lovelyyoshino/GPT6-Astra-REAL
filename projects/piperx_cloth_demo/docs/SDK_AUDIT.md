# 本机 pyAgxArm 能力审计（含 v0.7 单爪初始化）

## v0.8：返回已有六轴零坐标

厂家PiPER/default `move_j([0]*6)`经自动模式设置产生0x151和0x155/156/157；只修改已审计的内存速度缓存为1，避免`set_speed_percent`另发帧。零目标三帧各8字节全零；不调用编码器设零、夹爪、使能、停止、复位或失能方法。独立发送守卫限制selected arm/当前线程/精确帧顺序且总发送上限4，初始化与清理亦受限。厂家SDK只接受目标，不证明当前安装的整段无碰撞；本机运动停止仍未验证。软件测试必须使用假CAN，并与实机到位结果分开记录。

审计日期：2026-10-04；本记录核对源码、已有采样及无硬件测试，没有运行实机运动示例。
结论：新版提供可复用的纯 FK 和 TCP 计算，但未找到“输入任意末端目标、无运动返回逆解”的接口。
这不影响厂家控制器执行末端目标；它限制的是运动前可以声称完成了哪些检查。

## v0.7.1：单爪按实测开口初始化

姿态稳定检查改用厂家ZYX RPY转换的单位四元数间旋转角；整体旋转阈值0.003rad，其他关节/XYZ/开口守卫保持。欧拉分量差仅作诊断，不作为物理旋转角。没有修改SDK。

### v0.7原始接口

本机 `agx_gripper/default/driver.py:368` 的 `move_gripper_m` 将开口乘1e6、名义力乘1e3后调用厂家 `ArmMsgGripperCtrl(status_code=1)`，默认 `set_zero=0`，仅一帧0x159。消息定义将状态1标为 enable/width；未发现5mm下限或隐含臂运动指令。当前2.8mm、名义力0.2的预期帧数据为 `00000af000c80100`。原双爪准备工具的5mm下限不能表述为厂家硬件限制。

新单爪工具只接受臂名、保持实测当前开口，固定限制0–70mm输入并检查发送前后漂移，另一臂全发送封锁。此范围不是行程或力度标定；API编码可行也不证明实机准备成功。当前位置与使能同帧可能微调夹指，故须现场空爪、无接触、看护及新的使能/开口反馈。关节名义范围、抓取守卫和一般保持停止资格不受该准备工具改变。

## v0.6：用户指定旧工程的启动方法复核

2026-10-04 用户明确要求仅重新使能右臂并参考 `/home/agilex/GPT6_bash_jiang` 自行完善。只读核对其 [start_2_piper.sh](/home/agilex/GPT6_bash_jiang/start_2_piper.sh:40)：脚本会调用左右两臂使能服务，随后各自回零；其中左USB绑定也早于当前现场绑定。没有运行或修改该脚本。

它调用的[旧 ROS 使能服务](/home/agilex/piper_gpt/src/piper_ros-noetic/src/piper/scripts/piper_ctrl_single_node.py:474)每轮发送 `EnableArm(7)` 并发送闭爪位置目标，最多循环约5秒；此处不能直接视为纯使能，更不适用于仅右臂的一次请求。[旧回零](/home/agilex/piper_gpt/src/piper_ros-noetic/src/piper/scripts/piper_ctrl_single_node.py:549)与[旧急停](/home/agilex/piper_gpt/src/piper_ros-noetic/src/piper/scripts/piper_ctrl_single_node.py:521)也不纳入新入口。

现用厂家 [enable(255)](/home/agilex/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/piper/default/driver.py:764)将关节数6加1编码为电机选择7，经[厂家编码器](/home/agilex/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/piper/default/parser.py:464)发送 `0x471 0702000000000000`。其返回值立即读取接收缓存，不能因返回 false 自动重发。沿用旧成功控制链[逐驱动发送后时间戳确认](/home/agilex/piper_right_pick_demo/scripts/direct_sdk_pick.py:145)的原则，不照搬该脚本的多次重发。

新 `robot_startup_arm` 复用本项目已审计的模式/使能编码、被动连接、精确发送白名单和新反馈核验；仅选中臂各一次 `0x151` 与 `0x471`，另一臂所有发送封锁并保持原状态。无末端/关节/夹爪目标；角度越界只保留原反馈，启动不修复或扩大限位，且不解锁任务运动。原双臂启动、任务工具及厂家SDK均未改动。使能可能影响物理状态，结果须与实机后续反馈分别核对。

## 版本与来源

- 源码：`/home/agilex/pyAgxArm`；[版本文件](/home/agilex/pyAgxArm/pyAgxArm/version.py) 标记 `1.0.0`。
- Git HEAD：`841a625f5f4920e776f20b934eb13048b747e6d0`。版本结论只针对本机这份源码，不代表所有同名安装包。
- 审计时 Git 显示 CAN 激活脚本有本地修改，`GPT-sdk/`、`questions.md` 未被跟踪；不能称整个工作树为未经修改的厂家发行版。
- 型号与固件分别配置；安装新 Python SDK 不会自动升级控制器固件。

## 可以无运动使用的计算

| 入口 | 输入与输出 | 已检查的边界 |
| --- | --- | --- |
| `pyAgxArm.utiles.mdh_kinematics.get_mdh("piper_x")` | 返回型号 MDH 参数，长度 m、角度 rad | 参数表访问，不建立通信 |
| 同模块 `fk_from_mdh(mdh, joints_rad)` | 6 个关节角 rad → 基座下法兰 `[x,y,z,roll,pitch,yaw]`，m / rad | 纯正运动学，不发送 CAN，不求逆解 |
| `robot.fk(joint_angles)` | 同上 | 内部调用同一纯函数；仅做 FK 时无需创建硬件驱动 |
| 厂家 TCP / 法兰转换 | 由已知法兰到 TCP 的偏移进行位姿转换 | 只处理已提供的偏移，不估计本机指尖位置 |

实现依据：[纯 FK 源码](/home/agilex/pyAgxArm/pyAgxArm/utiles/mdh_kinematics.py:47)、[驱动 FK 入口](/home/agilex/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/core/arm_driver_abstract.py:462)、[型号参数](/home/agilex/pyAgxArm/pyAgxArm/api/constants.py)。
姿态采用 ZYX 的 RPY 约定；输出是法兰位姿，不是带着现场指垫、相机或衣物的碰撞模型。
纯 FK 本身不保证关节输入在有效限位内，也不检查碰撞、速度、奇异性或动力负载；这些必须分别报告。

同一厂家函数、六关节全零输入的型号对照（位置 m、RPY rad，数值已四舍五入）：

| 型号 / 历史记录 | 法兰位姿 |
| --- | --- |
| `piper_x` 纯 FK | `[0.096897,0,0.216827,-1.483704,0,-1.570796]` |
| `piper` 纯 FK | `[0.0561275,0,0.2132663,0,1.4835299,0]` |
| 历史右臂全零关节实测 | `[0.056127,0,0.213266,0,约1.483512,0]`，由旧 raw 单位换算 |

历史依据：[同一采样中的零关节与位姿](/home/agilex/piper_right_pick_demo/runs/passive_right_20261003T220927_145802.json:430)。后者更吻合 `piper`。2026-10-04 用户另明确确认两臂均为 PiPER，当前配置已更正；两臂当前关节的厂家 piper FK 位置与控制器反馈仅相差数微米。固件、工具几何及现场有效限位仍需分别核验。

## `get_ik_joint_angles()` 的真实含义

该函数没有候选位姿参数；读取控制器为已经提交的笛卡尔目标计算的关节解反馈，单位 rad。
厂家文档明确它只在 `move_p()` 后可用，要求 `PiperFW.V188` 或后续相应驱动、固件至少 `S-V1.8-8`。
实现从接收缓存读取 CAN `0x2AA`、`0x2AB`、`0x2AC`，无可用反馈时可返回 `None`；反馈目标角也不等于实测关节角。
依据：[V188 实现](/home/agilex/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/piper/versions/v188/driver.py:228)、[固件能力表](/home/agilex/pyAgxArm/docs/piper/firmware_reference.md:62)。
因此不能先调用 `move_p()` 再读取它，并把这称为“不运动预演”或无副作用 IK 检查。
历史右臂记录为 `S-V1.6-5250409`，见 [只读固件报告](/home/agilex/GPT6_bash_jiang/realsense_calibration/data/right_firmware_20261004_010058_1791046858302824691/report.json:134)。
该记录不能证明支持 V188 IK 反馈，也不能代表当前左右两臂固件；两臂应分别读取、核对。不得仅改驱动版本字符串来假装具备能力。

## 提交末端目标和离线求解的区别

- 旧 `EndPoseCtrl` 把末端目标编码为三条笛卡尔控制帧；单位为整数 0.001 mm / 0.001°，运动模式另行设置。
- 新 `move_p` 接受法兰 m / rad 位姿，按配置设置运动模式并发送目标；厂家控制器负责逆解与执行。
- 两者都可作为“模型计划 → 厂家逆解 → 实机运动”的入口，都不是仅返回可达性或关节角的 Python 数值求解器。
- SDK 返回、CAN 发出、控制器接受、实际到位是不同事件；不能仅根据 `motion_status == 0` 判断目标成功。

实现依据：[旧接口](/home/agilex/.local/lib/python3.8/site-packages/piper_sdk/interface/piper_interface_v2.py:2645)、[新接口](/home/agilex/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/piper/default/driver.py:958)。

## 本地 GPT-sdk 目录不构成求解器

`GPT-sdk/test.py`、`test0.py` 是连接/读取实验；`motion.py`、`sample0.py` 包含使能、关节目标和失能等顶层动作。
这些是本机额外实验 demo，不是当前 Git 跟踪的厂家求解库；目录名不能作为功能或安全依据。
它们没有提供独立逆解算法；部分示例仅凭运动状态判断完成，或把失能称为安全退出，不能直接用于新平台。
新平台不导入、不运行这些文件；只取用经过审查的厂家接口和纯计算函数。

## 可复用的昨天成功控制链

`piper_right_pick_demo/runs/pose_batch_20261003T213623_137631/` 是完整抓放反馈记录；用户另明确报告抓取成功。记录标记 `protocol_completed=true`、`phase=complete`，自动视觉字段仍保留 pending，不能否定用户的现场报告。
该轮旧 `piper_sdk 0.6.2 / C_PiperInterface_V2` 输出 `MotionCtrl_2(1,2,5,0)` 与 `EndPoseCtrl`，运行时 `external_ik_calls=0`；[实际输出](/home/agilex/piper_right_pick_demo/scripts/sdk_pose_batch.py:151)已经是厂家控制器逆解，不是 Python 数值 IK。
[使能流程](/home/agilex/piper_right_pick_demo/scripts/direct_sdk_pick.py:107)核对绑定和静止、发 `EnablePiper()`，要求六个使能反馈时间戳均在发送之后；代码允许有界重发，该成功记录实际只发一次。它的起点已模式 1 且已使能，不能直接证明重启待机的启动过程。本轮新工具复用其协议和新反馈确认原则，并完成两臂待机启动实测。
该成功流程的失败处理只封锁后续 TX 并继续观察，不自动急停或失能，也未证明能取消旧目标。[实现](/home/agilex/piper_right_pick_demo/scripts/direct_sdk_pick.py:418)与后来 `model_direct_vendor_ik` 中自动快速急停导致下落的险情是不同流程，须分别记录。

## 重启后通用启动

`set_motion_mode` 可在 SDK 层发送模式帧，但仍须实际反馈确认。`enable(255)` 实际发 `0x471 0702000000000000`，然后立即返回缓存使能位；返回 false 不等于已被拒绝，不能据此盲目重复。`move_gripper_m` 同时带开口目标和使能，无纯夹爪使能接口。
v0.4 固定启动工具先在双臂均未使能时逐臂确认 CAN 模式，再逐臂一次厂家关节使能；本次两臂成功，夹爪使能位仍为 false。它不发送任何位置/夹爪目标，不验证运动中停止。结果见 [实机记录](../runs/startup_3d72f0070d4549f586e6379e0e8c8faa/result.json)。

## 停止能力与工具声明

新版 `electronic_emergency_stop()` 发送 `ArmMsgMotionCtrl(1)`；用户文档仍说明抬起的机械臂会以恒定阻尼下降。
`disable()`、`reset()` 文档也有掉电下落警告；`disconnect()` 只释放通信，不保证取消控制器已接受的运动。
本地依据：[停止实现](/home/agilex/pyAgxArm/pyAgxArm/protocols/can_protocol/drivers/piper/default/driver.py:835)、[停止与复位文档](/home/agilex/pyAgxArm/docs/piper/piper_api.md:1656)。
此前旧接口快速急停后的实际下落见 [险情复盘](/home/agilex/piper_right_pick_demo/baselines/model_direct_vendor_ik/INCIDENT.md)。已找到停止候选线索，但本机保持能力仍未验证。
协议有 `0x150` 的 `track_ctrl` 暂停/终止字段，拖动示教也有单独暂停字段；存在字段不证明适用于在线 MOVE_P/L 或能保持姿态。[本机消息定义](/home/agilex/pyAgxArm/pyAgxArm/protocols/can_protocol/msgs/piper/default/transmit/arm_motion_ctrl.py:24)
厂家 ROS2 的停止回调读取当前关节角，再按模式发关节目标；这是另一种候选策略，不是电子急停，也未验证本机固件、新鲜度要求或停止距离。[厂家回调源码](https://github.com/agilexrobotics/agx_arm_ros/blob/ros2/src/agx_arm_ctrl/agx_arm_ctrl/agx_arm_ctrl_single_node.py#L982)
本版不下发这些候选停止。真实 [SDK 后端](../robot_tools/backend.py) 已实现 move_p / move_l / 独立夹爪调用，[执行器](../robot_tools/execution.py) 负责监测和记录；无硬件测试不解除本站暂停。

| 工具能力声明 | 本阶段应报告 |
| --- | --- |
| 厂家纯 FK、已知 TCP 转换 | 可用；型号正确性与偏移真实性另行核验 |
| 任意末端目标离线 IK | 本机已审查接口未提供；不得标为通过 |
| 厂家控制器 IK 反馈 | 固件有门槛，且先有运动目标；本阶段不以它做预演 |
| 双臂、环境与附加工具碰撞检查 | 未实现 / 未验证；不能由 FK 成功推定通过 |
| 安全保持停止 | 未验证；不暴露假 hold |
| SDK 运动与夹爪派发 | 实现已存在并有模拟故障测试；本站 hold 未验证，后端硬拒绝，配置不能解锁 |


## v0.9 单臂监督动作源码复核

2026-10-05：新增single_supervised_actions.py和两个固定入口。仅做源码人工复核、独立审查及AST语法检查；25个schema名称唯一。用户要求取消模拟测试，本次未运行测试/模拟，也不将此前446项结果归于新增代码。原双臂工具、SDK和完整计划门禁未改。具体记录见`runs/maintenance_single_actions_20261005/review.json`；实机动作单独留证。
