---
name: piper-atomic-runtime
description: 审查或接入 Piper 的 L1 原子运行时，核对新观测、单步准入、有限末端或夹爪发送、独立回执及故障锁存；不重新规划任务或自动启动设备。
---

# L1 宿主原子契约

这是 L1 总路由；25 个可单独调用的原子 Codex 技能见 [可见索引](../../../ATOMIC_SKILLS.md)。L3 当前任务和 L2 当前操作决定本轮需要哪个原子，避免把全部原子技能读进上下文。在 `projects/piper_right_pick_demo` 下按名称查询：

```bash
PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic move_eef_once
```

契约及纯校验函数见 [fast_pipeline.py](../../../projects/piper_right_pick_demo/src/right_pick/fast_pipeline.py)；物理适配与既有资格检查由 `fast_ros.py`、`fast_safety.py`、`fast_qualification.py` 承担。原子接口描述不产生物理授权。

每周期由宿主顺序完成新观测、一个提案、准入、一次发送、回执和下一观测证据。`move_eef_once`/`set_gripper_once` 位于 `dispatch_once` 内部，不能在它之外再发一次。模型仅在 decide 阶段调用一次；host guard、计数、回执和日志无需额外模型推理。

- 新观测保留设备身份、递增序号、真实时间和场景 ID；不能给旧 RGB 换时间戳。
- 到位只证明驱动事务结果，视觉才证明物体行为。回执缺失、部分发送、不确定结果触发全局锁存，不自动重发。
- 已明确授权的单臂维护隔离例外：首次任务前，仅另一失能空爪的一条坐标校正命令完整发送、终态及发送范围可审计但确认失败时，`maintenance_scope` 保留原故障，只允许禁止对侧 TX 的单臂入口。它不追认置零成功、不重发、不删除故障；未知/部分发送、任务故障或新动作失败仍锁存。普通单臂试夹可由同一目标帧使能，不另行要求置零或空爪初始化。详见[在线接续分流](../piper-task-pipeline/references/online-readiness.md#按当前反馈选择已有准备入口)。
- 宿主资格、场景通道、单位与驱动语义必须来自当前 Piper；ARX5 CAN、坐标、保持和停止语义不能照搬。
- 关闭客户端不能证明机械臂停止；保留驱动生命周期与独立观测的既有边界。

纯校验函数只校验传入结构与一致性，不能独立认证物理事实。L0 必须提供实际测量和可核验记录；未实现的原子能力返回 unavailable，不模拟成功。
