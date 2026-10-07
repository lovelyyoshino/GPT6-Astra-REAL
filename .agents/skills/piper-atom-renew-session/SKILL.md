---
name: piper-atom-renew-session
description: 核对 Piper 有界控制会话与心跳续期，保持原任务总预算。
---

# renew_session

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic renew_session`（项目目录下）。续期只延续经过核验的当前控制会话，不重置 L3 总时限、调用数或阶段；失败在下一次派发前退出。
