---
name: piper-arm-coordination
description: 审查 Piper 双臂任务或主臂执行辅臂观察的角色、共同场景、保持回执、联合准入、发送和故障传播；当前用于离线契约与适配审查，不代替双臂实机执行器。
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

当前 Piper paired 仅校验后顺序 near-time 发送，未接入受验证的双臂保持/发送执行器。fast CLI 对协同模式在资源打开前拒绝；配置示例与离线 recipe 不改变此状态。适配缺口见 [协同审查](../../../projects/piper_right_pick_demo/docs/ATOMIC_SKILLS_AND_DUAL_COORDINATION.md)。
