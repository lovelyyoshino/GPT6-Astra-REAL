# 右臂抓放实验与控制基线

新增独立模式 **`astra_fast_closed_loop`**：当前phase局部决策、紧凑RGB输入、单动作schema、阶段相机/reasoning策略和逐步计时。已完成已有照片+mock回放，并通过真实三路RGB、右臂ROS反馈与Astra的 `live-check`：两次模型决策分别9.16秒、11.31秒，实机命令0条。真实运动仍等待超时保持与现场限值资格，尚无优化后抓取性能数据。详见[审计与运行说明](docs/astra_fast_closed_loop.md)。

本工程保留两种任务决策方式的实机记录。先阅读 [基线索引与对照](baselines/README.md)，再按需要查看完整控制链和原始证据。

| 基线 | 工作方式 | 归档结果 |
| --- | --- | --- |
| [精细程序控制](baselines/right_pick/README.md) | 视觉几何与轨迹代码生成任务，SciPy 求逆解，SDK 发送关节目标 | 103 段计划，60 段发送，59 段到位；尚未闭爪 |
| [模型直接规划与厂家逆解](baselines/model_direct_vendor_ik/README.md) | 模型给阶段和末端位姿，固定执行器调用厂家末端控制 | 展开、张爪、转向完成；接近超时后危险下落，尚未闭爪 |

最新实验的 [完整控制链](baselines/model_direct_vendor_ik/CONTROL_CHAIN.md) 说明模型如何给出整段动作，以及厂家 SDK 和控制器怎样执行；[险情复盘](baselines/model_direct_vendor_ik/INCIDENT.md) 记录自动快速急停后的下落和尚未解决的安全问题。

**当前自动实机入口已暂停。** 基线里的原版源码、参数和录像用于审阅比较，不应直接回放历史位姿。当前暂停记录为 [vendor_execution_hold.json](runs/vendor_execution_hold.json)。

上述历史基线采用整段计划与固定执行器。新fast模式改为每轮当前phase的一个局部动作，关键位置重新看图；底层运动安全资格独立验证。两种模式的结果和计时口径须分别审阅。

原精细控制基线保持原样。文档中的“完整计划”与“实际执行完成”须分开理解；本页列出的两轮归档样本均未完成闭爪抓取。
