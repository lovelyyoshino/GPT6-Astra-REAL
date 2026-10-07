# 模型直接规划与厂家逆解的右臂实验基线

本目录保留 2026 年 10 月 4 日北京时间约 07:15—07:16 的实机尝试。用户要求模型直接根据相机画面、任务描述和机械臂状态给出完整抓放过程，逆解交给厂家控制器，允许用固定控制代码完成发送、反馈检查和录像。

**本次结果是失败并发生危险下落：展开、张爪、转向已完成，接近目标未到位，尚未执行下降抓取或闭爪。** 超时后的自动快速急停伴随约 159 mm 的 SDK 末端参考点下降。当前工作工程已暂停该实机入口；本归档不是可直接重跑的安全示例。

## 这次实验的意义

与 [精细程序控制基线](../right_pick/README.md) 相比，主要变化发生在任务决策和目标生成环节。模型给出“先展开、转向、接近，再抓起并放到旁边”的阶段以及六维位姿参数；代码不从图像计算红块坐标，不生成抓取策略，也不求逆解。厂家控制器接受了部分模型目标并产生真实运动。

这仍然使用了 Python 相机驱动和通用执行器。区别在于任务过程来自模型输出的数据，而不是为该红块再写一套专用抓取程序。本次新增的执行层还包含局部环境、右臂绑定及速度范围等限制，并非已经适配任意机器人。

这种分工为更换物体、摆位或任务提供了可复用的接口基础。**可泛化是研究方向，尚不是本次已经证实的能力。** 本次参数参考了此前的真实末端位姿经验；单次失败不能证明无需坐标约定、无需安全约束，也不能证明新任务零修改即可成功。

## 阅读与证据入口

| 要了解什么 | 文件 |
| --- | --- |
| 模型怎样给出动作，SDK 怎样执行 | [CONTROL_CHAIN.md](CONTROL_CHAIN.md) |
| 为什么发生危险下落，哪些问题尚未解决 | [INCIDENT.md](INCIDENT.md) |
| 程序实际接收的完整计划 | [run/plan.json](run/plan.json) |
| 实际完成阶段、超时和停止结果 | [run/report.json](run/report.json) |
| 逐次反馈与分帧时间戳 | [run/feedback.jsonl](run/feedback.jsonl) |
| 两路完整录像 | [前视](run/cameras/front.avi)、[右腕](run/cameras/right_hand.avi) |
| 模型规划时参考的画面 | [前视](observations/planning/front.png)、[右腕](observations/planning/right_hand.png) |
| 本轮执行开始时的画面 | [前视](run/cameras/00_initial/front.png)、[右腕](run/cameras/00_initial/right_hand.png) |
| 原版执行器、SDK 反馈适配、相机驱动 | [执行器](code/scripts/vendor_pose_execute.py)、[SDK 适配](code/scripts/vendor_sdk_feedback.py)、[相机驱动](code/scripts/vendor_camera_record.py) |
| 文件来源与校验值 | [MANIFEST.json](MANIFEST.json)、[SHA256SUMS](SHA256SUMS) |

两路录像各 909 帧，15 fps 名义时长 60.6 秒；主机帧时间跨度均约 60.579 秒，详见 [frames.jsonl](run/cameras/frames.jsonl)。两相机没有硬件同步。录像是过程证据，不是本轮实时视觉安全反馈。

## 保留范围

```text
model_direct_vendor_ik/
├── README.md              结论、研究意义与阅读入口
├── CONTROL_CHAIN.md       模型、SDK、固件和执行器的真实分工
├── INCIDENT.md            超时停止后的危险下落复盘
├── planning/              模型计划、必要修订与直接 SDK 终端记录
├── observations/          规划时参考的两路 RGB、元数据和历史状态
├── run/                   本轮原始计划、报告、反馈、照片和完整录像
├── code/scripts/          与实机记录哈希一致的三份原版源码
├── code/tests/            当时的 23 项离线检查
├── evidence/              险情摘要、关键帧、厂家停止说明和入口暂停证据
├── environment.json       本机 SDK 与相机环境的版本核对
├── MANIFEST.json          原始来源与归档位置对应
└── SHA256SUMS             本目录文件校验清单
```

`run/` 和三份原版源码按字节保留。原 JSON 的绝对路径不改写，归档映射见 `MANIFEST.json`；原运行目录下的内容对应这里的 `run/`。目录时间 `T071525` 是本机北京时间，不能按旧基线的 UTC 命名习惯解释。

`planning/plan_from_zero.json` 与 `revision_departure.json` 记录本条路线如何从零位经过厂家拒绝和模型调整，最终到达本轮起点；终端记录也保留在同目录。`planning/model_selected_plan.json` 是事后标记撤回的模型参数表，**执行时的原始版本以 `run/plan.json` 为准**。

规划观测的原始元数据包含旧采集器写入的红色候选及深度文件路径。本轮没有运行旧视觉识别或几何模块，也没有用这些深度文件重新求目标；本归档仅保留作为参考的 RGB 与原始元数据，未复制未使用的深度数组。实际运动开始时右臂已离开零位，不能把历史腕部画面当成本轮实时腕部画面。

原版执行器保留了导致险情的超时停止逻辑，不能因离线检查通过就用于实机。当前工作文件与原版之间只增加了入口暂停检查，差异见 [execution_hold.patch](evidence/execution_hold.patch)，暂停原因见 [vendor_execution_hold.json](evidence/vendor_execution_hold.json)。本目录不提供实机启动脚本。

## 归档核对

以下命令只核对本地文件，不访问机械臂或相机：

```bash
cd ~/piper_right_pick_demo/baselines/model_direct_vendor_ik
sha256sum -c SHA256SUMS
```

[离线检查日志](evidence/offline_checks.log) 记录 23 项检查通过；其中模拟发送失败的 ERROR 是测试输入，不是该次实机报错。测试检查反馈逻辑和接口约束，未验证停机后的重力行为、桌面碰撞或任意位姿的可达性。完整复现环境说明见 [environment.json](environment.json)。

## 后续实验应保持的分工

固定相机和 SDK 工具，由模型给出可审阅的任务参数与阶段；新任务优先修改数据，不新增物体识别、抓放策略或数值逆解代码。执行层负责单位、设备绑定、反馈、速度和安全约束，厂家控制器负责运动求解。

恢复实机前必须解决本轮暴露的停止方式和工具到桌面的安全间隙问题。之后才适合在固定工具接口下，更换方块位置、物体外观及放置要求，分别记录任务完成率、人工干预和安全中断；这些扩展尚未执行。
