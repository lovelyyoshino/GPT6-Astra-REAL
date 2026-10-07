---
name: piper-atom-read-receipt
description: 区分 Piper 发送尝试、驱动接受、目标到位和物体任务结果。
---

# read_receipt

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic read_receipt`（项目目录下）。宿主读取独立反馈；缺失、部分发送或未知结果锁存并停止后续目标。客户端退出不等于机器人已停止，驱动到位不等于任务完成。

pair 的 move 回执另含 `motion_effect`：用派发前后反馈区分请求变化、实测变化及超带横移。目标落在普通到位容差内，仍可能没有可辨认的实际微步；沿请求方向移动也不能掩盖横向偏离。该字段只描述机器人响应，不能填作插头/插排的物体进展、抓牢或接触力证据。
