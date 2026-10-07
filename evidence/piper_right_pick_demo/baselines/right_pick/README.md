# 本次右臂控制基线

后续实验以这一次已实际发生的抓取接近过程为依据。直接阅读 [完整控制链](CONTROL_CHAIN.md)，无需先阅读历史诊断记录。

```text
right_pick/
├── README.md          本页
├── CONTROL_CHAIN.md   从相机、规划、求解到 SDK 执行的详细说明
├── run/               唯一一轮有效实机记录与两路录像
├── code/              本次普通控制链所需源码和依赖版本参考
└── SHA256SUMS         一份标准文件校验清单
```

## 直接查看

- 过程录像：[前视](run/camera_recording/front.avi)、[右腕](run/camera_recording/right_hand.avi)。每路约 56 秒。
- [控制记录与完整计划](run/report.json)：`plan` 是完整计划；`transmissions` 是实际发送；`stages` 是实际执行与到位结果；`trace` 是反馈。
- [23 mm 开口时的初始观测](run/camera_recording/00_initial/observation.json)、[55 mm 开口后的准备观测](run/camera_recording/01_prepared/observation.json)：图像、深度和内参均在各自目录内。
- [现场终端记录](run/terminal.log)、[视频逐帧时间](run/camera_recording/video_frames.jsonl)。

实机已经打通使能、观测、几何估计、完整规划、求解、SDK 发送和反馈确认。完整计划 103 段，实际发送 60 段，59 段确认到位；尚未闭爪。以此作为用户认可的运动过程基线，不将未执行的后续动作记作已完成。

## 保留范围

只保留 `pick_attempt_20261003T194811_102398` 这一轮的 20 个原始记录文件。最终报告已包含完整计划，因此省略重复的 `planned_attempt.json`；不保留早期失败运行、历史诊断、离线试算、重复抽帧和压缩包副本。本次报告自身的真实末态仍保持原样。

`code/` 保留正常抓放分支的 13 个必要文件和 2 份依赖版本文件。两个带 `step` 的模块包含仍被使用的 SDK 接口、反馈和发送检查，属于当前依赖。源码是事后整理版本：到位门限已经由 0.15° 改为 0.3°，修改未实机复测。本基线不收录未实测的续跑功能；源码中的可选 `--resume-from` 分支不属于此副本支持的入口。

`run/` 文件逐字节复制，原 JSON 的绝对路径不改写。它们原先都在 `~/piper_right_pick_demo/runs/pick_attempt_20261003T194811_102398/`，本目录对应位置是 `run/`；完整计划里指向本工程脚本的路径，对应 `code/`。需要迁移时在工作副本中处理路径，基线作为只读参考。

标准校验，无需专用归档程序：

```bash
cd ~/piper_right_pick_demo/baselines/right_pick
sha256sum -c SHA256SUMS
```

## 后续使用方式

每次实验只给出任务和场景，由高层生成目标位姿与阶段顺序，现有求解器或厂家固件完成逆解，再经 SDK 执行并返回反馈，不再为每个任务新增脚本。详见 [控制链中的目标分工](CONTROL_CHAIN.md)。

这需要固定的相机、求解器和 SDK 调用入口，或者已有现成工具提供这些接口；SDK 本身不负责图像理解和自然语言任务推理。当前保存的是已验证的关节控制实现，通用工具入口和 `EndPoseCtrl` 固件逆解路线还未完成实机验证，本次整理不增加新的执行代码。
