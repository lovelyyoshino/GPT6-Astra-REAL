---
name: piper-atom-check-fresh-observation
description: 校验 Piper 本轮图像和状态的设备身份、递增序号、新鲜度与多路时差。
---

# check_fresh_observation

查询 `PYTHONPATH=src python3 -m right_pick.fast_task_pipeline --atomic check_fresh_observation`（项目目录下）。只用宿主提供的原始身份、帧序号与时间，拒绝陈旧或跨场景混合观测；此契约不能自行认证传入数据的真实性。
