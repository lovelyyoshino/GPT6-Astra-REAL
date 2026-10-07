---
name: piper-atom-read-receipt
description: 区分 Piper 发送尝试、驱动接受、目标到位和物体任务结果。
---

# read_receipt

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic read_receipt`（项目目录下）。宿主读取独立反馈；缺失、部分发送或未知结果锁存并停止后续目标。客户端退出不等于机器人已停止，驱动到位不等于任务完成。
