---
name: piper-atom-verify-return
description: 核对 Piper 任务本轮初始参考、完整末端位姿、关节和静止回位状态。
---

# verify_return

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic verify_return`（项目目录下）。参考必须来自本轮初态；任务完成和回位分别报告。现有夹笔实机 runner 尚无自动回位，不能由此契约推断已经回位。
