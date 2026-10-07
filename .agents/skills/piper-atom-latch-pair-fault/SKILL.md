---
name: piper-atom-latch-pair-fault
description: Piper 双臂任一侧发送或回执不确定时锁存整对并停止新目标。
---

# latch_pair_fault

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic latch_pair_fault`（项目目录下）。记录两侧独立回执与故障原因，不重发、不靠新 run-id 清除。锁存逻辑为离线契约；客户端退出不证明两臂停止。
