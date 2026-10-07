---
name: piper-atom-prepare-held-side
description: Piper 一臂动作前核对另一臂静止保持的独立当前回执。
---

# prepare_held_side

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic prepare_held_side`（项目目录下）。null/held 侧须有同场景、同宿主的独立保持证据；Piper 当前尚无已验证 powered hold，不能把 timeout 或不发命令当作保持资格。

持久 pair host 可从连续新反馈签发 owner/scene/peer 绑定的 `retained_target_stationary` 回执，发送前再次检查，未命令部件的锚点不逐步重置。该回执不等于接触负载保持。核对的是**本步静止侧**：左臂首次建立接触时，核对右臂静止；右臂拔出时，才需要左臂固定插排的实际证据及适用带载支撑。不要要求动作侧先完成接触，才允许建立接触。按 [接触与微调](../piper-manipulation/references/bounded-contact.md) 分流；当前适配器的具体缺口见 [PAIR_HOST.md](../../../projects/piperx_cloth_demo/docs/PAIR_HOST.md)。
