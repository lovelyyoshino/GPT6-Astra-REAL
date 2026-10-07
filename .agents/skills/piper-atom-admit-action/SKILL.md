---
name: piper-atom-admit-action
description: 对 Piper 单臂候选动作做当前状态、阶段、单位和预算准入。
---

# admit_action

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic admit_action`（项目目录下）。由宿主用新实测状态和当前 L2 限额检查提案；拒绝只返回原因，不发送。模型的安全判断不能替代宿主准入。
