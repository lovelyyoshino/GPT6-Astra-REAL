---
name: piper-atom-move-eef-once
description: Piper 已准入的有限末端目标动作类型，单次派发且核对整臂风险。
---

# move_eef_once

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic move_eef_once`（项目目录下）。这是 `dispatch_once` 内的动作类型，不是第二次发送入口。现有右臂夹笔链与独立持久 pair 宿主分别按实际适配范围使用；通用契约不自动连接硬件。端点限制不能替代整臂通道检查。

接触微调由 [L2 接触流程](../piper-manipulation/references/bounded-contact.md) 根据刚获得的物体响应提出一个更小目标，沿用原宿主速度、步长与故障处理。用真实操作类型准入，不把拔插改成接近来绕过接触接口拒绝。
