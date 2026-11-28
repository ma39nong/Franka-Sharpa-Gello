# GELLO 与 DAgger 的 rosbag 契约及保留差异

核查日期：2026-11-28。适用于本工程统一入口的 GELLO/Sharpa 采集与
`LTT/franka3-sharpa-pi05/deploy/fr3_sharpa_control/ui.sh` 的 rosbag 采集。
旧 Wuji、O30i/G20 录制器不在本次对齐范围。

本次只调整 rosbag 数据保存；唯一同时作用于 MCAP 的行为是 DAgger
“上一段完成质量分类后才能开始下一段”的门槛。MCAP 通道、编码、封口与完成判据不变。

## 已对齐的内容

动作类话题仅保存以下三条，类型均为 `sensor_msgs/msg/JointState`：

| Topic | 含义 |
|---|---|
| `/teleop/validated_arm_commands` | 安全网关验证后的臂指令，按关节名解释左右臂 |
| `/sharpa/left/command` | 左手实际提交给 SDK 的 22 关节目标 |
| `/sharpa/right/command` | 右手实际提交给 SDK 的 22 关节目标 |

不保存 `/teleop/arm_commands`、`/openpi/executed_joint_targets` 或硬件命令总线。
DAgger 的 SDK 下发链路没有独立手命令 ROS 发布者，因此录制端只读订阅已有的
58 维下发记录，按完整关节名校验后提取两只手，以表中的话题名称写入 bag。
其中的臂目标不入档；录制端不向 ROS graph 发布拆分命令，不修改控制链。
`/teleop/arm_command_status` 是状态证据，继续保留。

三路 RGB、双臂/双手实测状态以及原有可选深度、触觉、诊断与执行上下文继续保留。
“只录制三条动作话题”不限制这些数据。

两边均配置三路 `/cam{0,1,2}/color/camera_info`、`/tf` 和 `/tf_static`。
开录前必须收到三路 RGB 内参及真实静态 TF，并加载含 cam1、cam2 的真实腕部外参。
标定只读使用各自现有 MCAP profile 的 `static_extrinsics`，不修改 profile。
缺少配置或快照时拒绝开录，不用单位矩阵代替现场标定。

两边均保存开录时刻的 RGB CameraInfo 和标定后的静态 TF 快照，录制期间的静态 TF
也应用同一腕部标定。内参保留原消息 header；快照的 bag 时间为开录时刻。
不改写原始图像、数值状态或动态 `/tf`。
`wrist_extrinsics.json` 位于 bag 根目录，结构为 `{"static_extrinsics": [...]}`。

本工程在 `collection_state.json` 增加开录时冻结的 `task_metadata`，保留 GUI
提交的任务 ID、名称和英文描述等字段；其他字段及完成判据沿用原规则。
DAgger 的 `episode.json`、`run_summary` 与任务字段位置不变。

## 仍保留的差异

下游不能把两边的生命周期字段或自动完成状态直接等同。

| 项目 | 本工程 GELLO/Sharpa | DAgger |
|---|---|---|
| 写盘后端及就绪 | 独立 `ros2 bag record`；启动不等待 writer 订阅/磁盘就绪握手 | 进程内 `SequentialWriter`，建立 writer 后激活；保留初始化后的传感器复查 |
| 标定快照写入 | 开录时冻结消息和时间；CLI 正常退出封口后补写 SQLite，再执行原有校验 | writer 建立后立即写入，再激活在线录制 |
| 静态 TF 处理 | 封口后按时间顺序修正已录制的 `/tf_static`，补入开录快照，更新原生 metadata 的计数及起止时间 | 接收时合并静态 TF 并应用标定，直接写入在线包 |
| 异常封口 | CLI 异常退出或补写前进程中断，不能保证标定快照完整；补写异常进入既有失败处理 | 沿用原有进程内写盘、关闭与失败处理 |
| 元数据布局 | `collection_state.json`，顶层 `task_metadata`、来源、milestones、校验等 | `episode.json` 中 `quality`、`run_summary`、诊断等；日期目录保留运行日志/样本链接 |
| 自动完成状态 | 原 `state=finalized` / `finalized=true`；保留必需流、频率、时间连续性、内容及在线检查 | 原 `COMPLETE`：已开录、无写入错误且各必需话题有消息；未增加 GELLO 的完整校验 |
| 时间窗口验收 | 沿用开头 1 秒、结尾 0.2 秒等配置及 source-header 校验 | 不加入对应裁剪校验；`COMPLETE` 不等于通过同一连续性验收 |
| 下一段门槛 | 原有分类成功、双臂 Home 成功、相机/传感器就绪等条件保持 | 上一段实际开始的 DAgger 样本必须完成 A/B/C/D 分类；不要求 Home。原有执行清理和并发 Home 互斥保持 |
| 评级与后续操作 | 原有质量归档、改评/弃用、自动报错、传输及删除规则保持 | 原有 revision/history/note、后台导出后归档及改评规则保持；分类完成可开始下一段，不必等导出结束 |
| 接收与缓存 | 原 CLI QoS 覆盖和 256 MiB 写盘缓存保持 | 原传感器 BEST_EFFORT/depth 30、可选话题可靠/depth 1000、静态 TF transient-local/depth 100 保持；验证后臂命令仍为原可靠/depth 1000 |
| 接收进度门槛 | 原有类型、亮度、传感器就绪与连续性规则 | 原本地接收新鲜度和进度检查；不改成 GELLO 的缓存/验收模型 |
| 附加上下文 | 原有相机健康等诊断主题、任务和完成标记 | 原有 `/dagger/frame`、`/dagger/events` 等接管/执行上下文；不为 GELLO 伪造这些数据 |
| 视频及运行记录 | 沿用本工程保存及手动后处理流程 | 沿用后台视频导出、日期记录和分类目录链接 |

GELLO 的快照补写方案已获用户确认，用来保留原写盘后端。补写只操作已关闭的 SQLite
bag，使用开录时冻结的时间戳，不向现场发布 TF。bag 首条快照不能证明 writer
在该时刻已成功接收图像/动作。

DAgger 新门槛通过持久化报告识别最近一段实际开始的 DAgger 录制，对 rosbag 和
MCAP 相同，UI 重启后仍生效。未真正开录的尝试不产生门槛，不要求补评更早历史积压。
该门槛不改变评级含义或自动完成状态。

## 验证与维护入口

- 隔离 ROS Humble 容器运行采集包及 DAgger 对齐测试：289 passed、228 skipped。
  跳过项包含未配置转换 oracle 和可选依赖的测试；未执行机器人真机验收。
- 新增 13 项对齐检查，包括两边真实 SQLite writer/reader 回读、动作话题、TF 标定
  和开录时间、外参 JSON、任务快照，以及分类跨格式和重启恢复。
- MCAP 模块 SHA-256 与本次 rosbag 修改前一致；现有 MCAP 回归包含在上述测试中。
- Qt 设置、操作 GUI、统一 GUI 与传输面板回归：42 passed。

本工程实现位于 `ros_ws/src/teleop_data_collector/teleop_data_collector/` 下的
`recording_backend.py`、`rosbag_recording_node.py`、`rosbag_calibration.py`。
DAgger 实现位于 `deploy/fr3_sharpa_control/` 下的 `rosbag_contract.py`、
`recorder_ros.py`、`quality.py`、`controller.py`、`record_cli.py`、`ui.py`。
两边分别新增 `test_rosbag_alignment.py`。

MCAP 契约与已有验证见 [MCAP_RECORDING.md](MCAP_RECORDING.md) 和
[MCAP_VALIDATION.md](MCAP_VALIDATION.md)。现场更改标定或完成判据时，应同步复核本页。
