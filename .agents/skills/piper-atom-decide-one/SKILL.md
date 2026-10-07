---
name: piper-atom-decide-one
description: Piper 当前单臂阶段基于新观测提出一个有限动作，限制一次模型调用。
---

# decide_one

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic decide_one`（项目目录下）。只输入当前 L2 操作、短状态、剩余预算及选定新 RGB；只输出一个结构化提案。无证据时选择有价值的一次观察或退出，不重新规划全任务；提案不等于准入。
