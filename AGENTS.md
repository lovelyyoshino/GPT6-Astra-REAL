# Piper 项目执行规则

## 已确认的使用方式

- 用户通过 Codex 进入本项目并完成控制；模型决策使用现有 Codex 登录和 `codex-cli`，默认 `--model codex`、`protocol=codex_cli`，不要求配置 API key。
- 本轮任务是本地优化、技能整理和离线验证，写入范围只限此交接包。实机操作必须对应用户实际要求执行的任务及当前现场准入。
- 已有明确目标及有效授权时直接推进；只询问传感器无法获得且影响目标或执行的必要信息。优化本身不反复确认、不循环 prepare。

## 按层调用

```text
L3 task pipeline -> L2 当前有界操作 -> L1 原子契约 -> L0 Piper 适配器
                         ^ 新观测 + 执行回执 + 阶段证据 |
```

- 首先使用 [.agents/skills/piper-task-pipeline/SKILL.md](.agents/skills/piper-task-pipeline/SKILL.md)。只读取当前层需要的技能；不要把全部手册放进每次决策。
- 25 个可单独调用的 L1 Codex 技能及其真实实现状态见 [ATOMIC_SKILLS.md](ATOMIC_SKILLS.md)；`.agents` 是隐藏目录，从本交接包根目录启动 Codex 才能发现这些项目技能。
- 从上级目录启动的旧会话不会向下发现本包技能；需新开 `codex -C /Users/pony.ai/Documents/文档/Piper_SingleArm_Handoff`，再用 `/skills` 检查入口。仅在工具命令里切换目录不改变会话的发现范围。
- L3 负责目标、初态、角色、阶段和总预算；L2 负责操作条件、动作范围、证据与局部预算；L1 校验和发送由宿主代码负责，不为每个原子步骤再发起模型会话。
- 常规周期至多一次模型提案和一次物理发送；执行回执不确定则锁存，禁止自动重发。
- 零状态变化的观察不算进展。达到已有观察、阶段、调用或时间预算就返回具体阻碍和退出原因；renew、恢复和会话重启不能清零任务总预算。
- 跨命令维护通用离线任务时使用 `right_pick.fast_task_session` 的同一 store/run-id；先读 current，再以 revision 和唯一 event-id 记录回执。重复 init 返回已有进度，不能靠更换 ID 自动开启新一轮。
- Codex 外层读取技能与调用工具；`fast_codex.py` 内层只做隔离的单步 JSON 决策，不读取这些手册或调用控制工具。不要同时开两个控制宿主。

## 实现边界

- 当前可执行源码仅为单右臂 `pen` fast runner；现场准入已撤回。本机有 Codex 不代表 ROS、相机、保持或路径已具备实机资格。
- ARX5 的 18 个任务、双臂和主臂执行/辅臂观察目前是离线可调用契约，尚无通用实机执行器。不能静默降级成单右臂夹笔或因配置存在就声称支持。
- 双臂一次只动一侧，另一侧需要当前独立保持回执；观察臂只改善视角，不能抓取、固定或支撑任务物体。
- close/disconnect、客户端退出和失能都不能当作已验证的停止。保留被动臂 TX-block 与故障锁存。
- 抓取、释放后独立稳定、受控回位分别记结果。历史坐标、源快照和 ARX5 成功记录不能成为本轮动作或资格。

## 本地验证

在 `projects/piper_right_pick_demo` 下运行：

```bash
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_fast*.py'
python3 -m compileall -q src
```

在交接包根目录运行 `python3 tools/verify_bundle.py`。修改后更新本地优化清单，保留 `metadata/source_manifest.json` 原始来源哈希。
