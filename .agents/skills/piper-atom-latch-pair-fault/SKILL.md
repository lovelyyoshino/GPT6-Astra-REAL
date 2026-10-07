---
name: piper-atom-latch-pair-fault
description: Piper 双臂任一侧发送或回执不确定时锁存整对并停止新目标。
---

# latch_pair_fault

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic latch_pair_fault`（项目目录下）。记录两侧独立回执与故障原因，不重发、不靠新 run-id 清除。

持久 pair host 已用 SQLite 保存唯一 owner、发送前 claim、预算与全对 fault；claim 后进程消失也视作不确定。部分发送、peer 漂移、异常反馈、客户端意外退出会锁住两臂新增目标，并阻止同项目旧动作入口绕过。原离线函数仍只校验合同；跨项目或不合作的 ROS/SDK 发送方仍需现有唯一宿主流程。客户端退出与软件 cancel 都不证明物理停止。入口见 [PAIR_HOST.md](../../../projects/piperx_cloth_demo/docs/PAIR_HOST.md)。

故障锁存后，仍打开的 pair 宿主会继续通过现有 RX 缓存读取反馈。查看 `robot_pair_status` 的 `fault_feedback`、采集时间、分片新旧和 `fault_feedback_read_state`；原动作线程尚未退出时诊断明确暂缓，不启动第二个宿主。该诊断允许显示失能、异常、旧帧或缺帧，不生成新的动作场景、保持回执或停止结论。故障后的 `robot_read_state` 也返回诊断范围；不能因重新读到健康反馈而解锁或重派。
