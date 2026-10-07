# PiPER 控制交接包：单臂闭环与分层技能

本包基于从 `agilex@192.168.2.25` 只读整理的 **2026-10-06 源码快照**，现已加入本地控制链优化。当前执行源码面向右臂夹笔放筒；ARX5 的 18 项任务已整理为分层离线契约，覆盖单臂、双任务臂和主臂执行/辅臂观察的角色与阶段。双臂与通用任务尚未接入实机执行器。

全部修改仅在此交接包内，未启动机械臂、相机、ROS 或真实模型，未修改参考项目与远端。原始来源哈希保留在 [source_manifest.json](metadata/source_manifest.json)，本地改动单列在 [optimization_manifest.json](metadata/optimization_manifest.json)。包内不含视频、原始 CAN 大数据、完整逐帧记录或登录凭据。

## 通过 Codex 使用

从本目录进入 Codex，读取 [AGENTS.md](AGENTS.md) 与 [任务技能入口](.agents/skills/piper-task-pipeline/SKILL.md)。[原子 Codex 技能可见索引](ATOMIC_SKILLS.md)列出隐藏在 `.agents/skills` 下的 25 个独立 `SKILL.md`。沿用现有 Codex 登录与 CLI，默认 `--model codex`、`protocol=codex_cli`，无需 API key。层次为 **L3 任务 pipeline → L2 有界操作 → L1 当前原子技能/契约 → L0 Piper 适配器**；宿主执行检查和回执，不为每个原子再调用模型。

从上一级目录启动的 Codex 不会发现这个子目录中的项目技能。用 `codex -C /Users/pony.ai/Documents/文档/Piper_SingleArm_Handoff` 新开会话，再在 `/skills` 中查找 `piper-task-pipeline`；终端和 Finder 查看隐藏目录的方法见[可见索引](ATOMIC_SKILLS.md)。

[分层技能与任务映射](projects/piper_right_pick_demo/docs/ATOMIC_SKILLS_AND_DUAL_COORDINATION.md)列出 18 个任务、20 个组合操作、25 个原子接口及实现边界。单右臂 fast 默认有 24 次模型调用、2 次连续未知观察和 900 秒预算；达到上限输出具体退出原因。实际 token 降幅和成功率尚待现场对照验证。

通用离线任务可通过 `right_pick.fast_task_session` 保存进度，供 Codex 跨命令读取当前阶段、记录宿主回执和进入观察分支。重复初始化或重启不会重置阶段与预算，重复事件不会再次推进。它维护任务账本，不发送设备命令。

## 先理解当前进度

- **有现场协助的夹笔放筒已成功**：2026-10-05 更换无隔板笔筒后的实验，采用 ROS 逐段执行，现场人员确认过前后对齐；释放、撤离后图像确认笔留在筒内。见 [结果摘要](evidence/piper_pen_repeat_video_20261005/new_holder_result.json) 和 [实验 PDF](evidence/piper_pen_task_report_20261005/夹取笔放入笔筒_实机实验报告_20261005.pdf)。它不是可直接重放的通用自动计划。
- **后续 fast 自动闭环代码已经存在，但未完成本轮抓笔**：最新保留记录中，一条约 +X 28 mm 的 P 目标实际引起较大的关节重构；客户端报错退出后，驱动继续完成已接受的有限目标。原客户端失败结果与事后确认到位分别保留，不能改记成自主任务成功。
- **当前复制到的 fast 实机准入已撤回**：参见 [fast_live_commissioned.json](evidence/piper_right_pick_demo/runs/fast_repair_20261005_182331/fast_live_commissioned.json)，其中 `physical_qualification.evidence_file=null`。同目录有 [command8 复核](evidence/piper_right_pick_demo/runs/fast_repair_20261005_182331/command8_posthoc_audit.json) 和撤回记录。
- 原工程 README、AGENTS 和部分文档累积了不同时间的状态。PID、boot ID、会话号、现场授权、旧目标都属于历史记录；本次未查询实时设备状态。特别是 `docs/astra_fast_closed_loop.md` 中较早的“仅 mock”描述已落后于当前源码。

## 文件布局

| 位置 | 内容和用途 |
|---|---|
| [projects/piper_right_pick_demo](projects/piper_right_pick_demo) | 当前 `right_pick/fast_*` 模型闭环、ROS 适配、相机进程、测试，以及早期夹取实现 |
| [projects/piperx_cloth_demo](projects/piperx_cloth_demo) | 名称保留历史叫法；当前任务配置指向夹笔放筒，包含通用工具、只读观测与冻结 ROS 接续入口 |
| [vendor/piper_ros-noetic](vendor/piper_ros-noetic) | 实际使用的厂家 `piper`、`piper_msgs` 源码、消息/服务定义、launch 和构建文件；未带仿真/MoveIt/描述模型等非本控制链包 |
| [vendor/piper_sdk_0_6_2](vendor/piper_sdk_0_6_2) | 实际 ROS 控制使用的旧 `piper-sdk 0.6.2` 安装源码及发行元数据，供源码对照；不是完整虚拟环境 |
| [vendor/pyAgxArm](vendor/pyAgxArm) | 新版 SDK 源码及许可证；用于部分工具/观测/FK，不能代替 ROS 链上的旧 SDK |
| [vendor/driver_with_health.py](vendor/driver_with_health.py) | 旧低速入口引用的健康遥测包装源码 |
| [evidence](evidence) | 49 份精选小记录：结果、实际 launch、少量命令示例、模型输入/输出和失败复核，另含一份 PDF、两张最终图片 |
| [metadata](metadata) | 源文件来源、版本、环境及完整性检查结果 |
| [ATOMIC_SKILLS.md](ATOMIC_SKILLS.md) / [.agents/skills](.agents/skills) | 可见原子索引；隐藏目录中有 4 个分层入口与 25 个单项原子 Codex 技能，按当前操作加载 |
| [OPTIMIZATION_GUIDE.md](OPTIMIZATION_GUIDE.md) | 优先阅读的代码入口和优化建议 |

## 工作链与推荐阅读顺序

当前 fast 链：

```text
right_pick.cli → fast_cli → FastLiveClosedLoop
  → 新 RGB + ROS 反馈 → 当前阶段的模型单步提案
  → schema / phase / 新鲜度 / 数值限制 / 实机资格检查
  → fast_ros → ROS topic/service → ros_resume_entry
  → 厂家 piper_ctrl_single_node → piper-sdk 0.6.2 → can1
  → 独立反馈与命令回执 → 记录 → 下一轮
```

Codex 先从任务技能选定当前任务和层次；审查执行实现时按下列入口读：

1. `projects/piper_right_pick_demo/src/right_pick/fast_live_loop.py`：完整真实循环、派发与反馈。
2. `fast_policy.py`、`fast_live_policy.py`：阶段、动作约束、视觉验证与结束条件。
3. `fast_codex.py` / `fast_model.py`：模型输入、结构化输出、超时和计时。
4. `fast_ros.py`、`fast_safety.py`、`fast_qualification.py`：底层合同、资格证据和失败处理。
5. `projects/piperx_cloth_demo/robot_tools/ros_resume_entry.py`：固定厂家驱动上的接续和发送守卫。
6. `projects/piperx_cloth_demo/tasks/put_pen_in_holder/task.json`：任务语义与历史记录。该文件约 163 KB，作为实际配置保留；其中历史状态不构成新任务起点。

源码中还保留了红块、杯子及专项恢复工具，方便对照。`scripts/ros_guarded_*` 等入口通常绑定某次现场状态及证据，不应当作通用夹笔入口。早期基线说明放在 `evidence/piper_right_pick_demo/baselines/`。

## 本机先做什么

解压后进入此目录，使用标准 Python 检查文件完整性：

```bash
python3 tools/verify_bundle.py
```

这只读本地文件，不访问硬件或模型，分别检查原来源哈希、本地优化覆盖与交接清单。本地已运行 fast 回归测试、语法与技能检查，最新结果见 [optimization_validation.json](metadata/optimization_validation.json)；远端历史测试不计入此次结果。

复核本地控制链修改：

```bash
cd projects/piper_right_pick_demo
PYTHONPATH=src python3 -m unittest discover -s tests -p 'test_fast*.py'
python3 -m compileall -q src
```

没有随包复制 Codex 登录文件、API key、SSH 凭据或 Conda 环境。上述回归使用假模型和假适配器；真实 Codex 调用沿用运行环境自己的登录。

## 运行环境与迁移边界

采集结果见 [remote_environment.json](metadata/remote_environment.json)。现场依赖并非单一 Python 环境：

| 用途 | 远端实际环境 |
|---|---|
| ROS 控制 | Ubuntu 20.04 / ROS Noetic、系统 Python 3.8、`piper-sdk 0.6.2`、`python-can 4.5.0`，需要编译 `piper_msgs` |
| 部分数值/旧控制脚本 | `aloha` Python 3.8，NumPy 1.24.4、SciPy 1.10.1 |
| RealSense 相机进程 | `pi0_infer` Python 3.10.18，NumPy 1.26.4、OpenCV 4.11.0.86、pyrealsense2 2.56.5.9235 |
| fast Codex 后端 | 当前源码精确检查 `codex-cli 0.160.0`，模型配置为 `gpt-6-astra`；需单独安装/登录 |

`pi0_infer` 中同时装有旧 `piper-sdk 0.4.1`，不能因为它能打开相机，就把它用于要求 0.6.2 的 ROS 驱动。`pyproject.toml` 未完整声明依赖，单独 `pip install -e` 不等于环境已经齐备。

现场配置 `*.local.json` 已保留，包括实际右臂 `can1 / USB 1-6.3:1.0` 和三相机身份；旧 example 可能仍写 `can2`。这些是机器绑定，迁移时重新核对。

代码保留了 `/home/agilex/...` 绝对路径，部分路径及源码哈希参与校验，例如 `fast_ros.py` 固定引用接续入口和厂家驱动。修改目录或控制源码之后，需要更新并复核对应合同，不能只用字符串替换就宣称真机可运行。来源到本包的完整映射已记录在 `metadata/source_manifest.json`。

ROS 工作空间只带源码，未复制 `/opt/ros`、`devel` 或系统二进制。典型原现场先加载 `/opt/ros/noetic/setup.bash` 和 `piper_gpt/devel/setup.bash`，再启动经过复核的唯一右臂驱动；真实启动会访问设备，本文不给历史状态自动接管或目标重放的承诺。所选现场 launch 在 `evidence/`，其中接续要求与初始化入口要求不同。

## 有意省略的内容

没有打包视频、完整 `runs/`、逐帧图像、原始 CAN 窗口、多 MB 资格轨迹、虚拟环境、Git 历史、缓存及构建产物。保留的小 JSON 中可能引用这些远端路径，按源证据索引理解即可。

尤其 `qualification_evidence.json` 等大文件没有复制；旧 boot/PID/驱动日志绑定也不能搬成新机器的许可。因此这是可追溯的**源码优化交接包**，跨机器实机复现还需要环境配置、现场验证和新的资格证据。

供应商源码保留原许可证和元数据。本次生成的 `SHA256SUMS` 与 `metadata/source_manifest.json` 是新的交接清单；原 `PLATFORM.sha256` / `SDK_SOURCE.sha256` 作为历史文件保留，未伪造更新。其中旧 PLATFORM 清单的 AGENTS.md 摘要已落后于实际源文件，以本次快照清单为准。
