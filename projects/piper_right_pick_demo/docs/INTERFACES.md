# 接口与边界

长度米，关节角弧度，内部四元数 wxyz；夹爪 width_m 表示两指总开口。位置动作明确使用 right_base 下的已标定 TCP，不混用 SDK J6。

`Action.from_dict` 接受 `observe/move_tcp/gripper/wait/stop`。move_tcp 要求 pose.position_m、pose.orientation_wxyz、speed_m_s、issued_at、ttl_s、calibration_version；gripper 要求 width_m 及同样的时效/版本。arm 只能为 right，额外字段及非有限数拒绝。

`validate_action` 检查有效期、观测时间、标定版本、几何是否改动、设备真实反馈/绑定/使能/故障、TCP/夹爪/工作区是否确认、平移旋转步长和速度。检查通过不表示轨迹避障已验证，也不能替代底层持续控制。

`RosRightArm.observe` 读取 JointState、PoseStamped、PiperStatusMsg。驱动可能重新给缓存数据打时间戳，因此不会声称设备数据新鲜；状态消息不能证明全部电机使能，enabled=null。move/stop 当前明确未配置。

`RosCameraRig.capture` 读取已有三路 RGB 与内参，可读取已对齐深度；未知单位、时间过期或不同步会拒绝。序列号配置不是现场身份确认。`RealSenseRig` 按序列号打开实际设备，保存 RGB、原始深度、米深度、内参、设备与主机时间。只读像素查询不会使用旧外参。

`Recorder` 为每个回合建立独立目录，记录实际配置、事件、模型请求用量及报告。usage 缺失为 unknown/null；cached 与 reasoning 是子集，不重复相加。新回合清空历史，模型历史不包含评分真值或详细思考。

当前可选模型接口只生成单步提案；完整真实持续执行循环尚未验收。合成回放始终标注 nonphysical，不冒充真实传感器或物理成功。
