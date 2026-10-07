---
name: piper-atom-observe-scene
description: Piper 闭环周期采集一次带身份、时间与机器人实测状态的新场景。
---

# observe_scene

在当前 L2 操作需要新场景时查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic observe_scene`（项目目录下）。输出 observation_id、RGB 视图和实测状态；不能用旧图换时间戳。采集失败消耗本轮预算并返回缺口，不触发动作。
