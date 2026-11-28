# MCAP 与 rosbag 采集

统一 GUI 右侧“本地采集设置”新增样本保存方式、头部深度、腕部深度和双手触觉。
首次默认 MCAP、头部及双腕深度开启、触觉关闭。选择立即保存，重启沿用。
两种格式均使用这些开关，三路 RGB 始终采集。腕部开关同时控制左右腕。

任务选择下方可填写英文任务描述；按任务保存。留空时使用任务名称。
录制开始后任务描述和采集选项冻结，停止并完成校验后恢复编辑。

## 本机准备与正常启动

本次已在本机复制了 DAgger 的实际模型、标定及色彩/时钟证据，放入
`~/.config/gello-teleop/mcap-assets/`，并生成 `mcap-profile.yaml`。
不需要从 DAgger 仓库导入 Python 模块。相同设备、相同安装的适用条件需保持；
设备序列号和视频原生尺寸不匹配会阻止采集。

在其他安装环境首次准备时：

```bash
# 在采集容器的 Python 3.10 中安装，不能使用主机其他 Python 的二进制扩展。
docker compose -f docker/compose.yaml run --rm --no-deps \
  -e PIP_INDEX_URL=https://mirrors.aliyun.com/pypi/simple \
  data-collection bash ops/setup/setup_mcap.sh

# 在主机复制已核验的参考配置及真实资产；输出独立副本，不修改参考工程。
/usr/bin/python3 ops/setup/prepare_mcap.py \
  --reference /home/descfly/LTT/franka3-sharpa-pi05/.deployment/mcap-recording.yaml
```

默认依赖目录为工程的 `.runtime/mcap-python`，已在本机安装并编译 RVL。
新 Docker 镜像也包含 Python 依赖；现有镜像由采集启动脚本加载独立依赖目录。
NumPy 固定为 1.26.4，兼容 ROS Humble 的 OpenCV 扩展。

正常操作仍使用两个独立入口：

```bash
# 遥操；默认不使能机械臂或手部输出。
ops/run/run_gello_full_teleop.sh --hand-source manus --hardware

# 独立采集器；根据 GUI 保存的选项决定是否要求深度和触觉。
ops/run/start_sharpa_arm_recording_with_tactile.sh --with-cameras
```

先选好传感器，再正常启动硬件和采集终端。改变保存格式无需重启硬件。
新勾选的深度或触觉尚未发布时，需在下一次正常硬件启动时加载。
本功能不在线重启相机，不另开 Sharpa SDK 连接。
关闭或重建共享相机前应先退出采集器，沿用原有设备占用保护。
共享相机独立于 GUI 存活，退出 GUI 不等于重启相机。首次启用本功能，或改变
深度开关后遇到“Shared cameras use a different deployment config”时，完成录制、
退出采集终端，并在设备空闲时执行 `docker compose -f docker/compose.yaml stop teleop-cameras`，
再使用正常入口启动。程序不会自动替换正在运行的共享驱动。

深度配置写入独立副本，不修改原始相机 YAML：头部 640×400@20，双腕
640×480@30，原生 Y16。已启动服务的配置不同会明确提示，不在线替换。

## 设置与接口

- 采集选择：`~/.config/gello-teleop/recording.json`。
- MCAP 配置：同目录 `mcap-profile.yaml`。
- 可通过 `COLLECTION_RECORDING_CONFIG`、`COLLECTION_MCAP_PROFILE` 覆盖位置；
  自定义位置须位于采集容器可读挂载目录内。
- 英文描述保存在原任务注册表的可选 `task_descriptions` 映射中。
- 控制接口新增 `configure_recording`，参数为
  `format: mcap|rosbag`、`head_depth: bool`、`wrist_depth: bool`、`tactile: bool`。
- `status.recording_settings` 返回能力与当前选项；`start.task_metadata` 接收
  `task_id`、`task_name`、`task_description`，并对本段快照。
- 旧采集器仍可选择 rosbag 使用原接口；选择 MCAP 会提示更新，不静默改为 rosbag。

## 保存内容

父目录内使用 DAgger 当前归档结构：

```text
assets/models/<hash>/
_shared/<hash>/devices.yaml
_shared/<hash>/calibration/
_shared/<hash>/configs/
pending/episodeXX/
A/episodeXX/
B/episodeXX/
C/episodeXX/
D/episodeXX/
```

每个 MCAP episode 包含 `data.mcap`、`metadata.yaml`、`episode.json`、
`channel_stats.json`、`manifest.json`、`checksums.sha256`，事件按需保存。
不增加日期运行目录、查看视频、拼接视频、快捷方式或单独的采集状态文件。

评级采用 `episode.json.quality`；本工程的 Home、传输就绪和连续性状态放在
`episode.json.collection`。元数据更新后重新封装清单及 SHA-256。
A/B/C/D 是人工分类，不代表 MCAP 校验结果或任务成功。

机械臂 action/command 来自安全网关验证后的目标，手部来自 SDK 下发目标，
分为左右臂、左右手四路标准 JointState。measured state 独立保存。
输入关节名和映射来自 `teleop_core.contract`。不写入 DAgger 策略/介入记录，
不把本工程样本标记为 DAgger。自定义控制消息仅在归档端转换为标准消息。

RGB 原生尺寸、H.264/medium、主相机 CRF18、双腕 CRF16、GOP30、无 B 帧；
深度为无损 compressedDepth RVL；触觉保留原图、形变、F6、原生 SDK 采样信息，
接触点按有数据时记录。触觉不是仅保留最新帧的诊断队列。

按 L 开始；再按 L 停止、排空队列并封口。之后的 Home 不进入该段。
故障保留 failed/partial 证据，不能生成假的 complete。单文件接近 10 GB 时
终止；本版本不实现分片或断续恢复。

## 混合目录与后台传输

可与 DAgger 选择同一保存父目录，使用相同编号锁和 `.episode_sequence.json`。
评级使用同文件系统的非覆盖移动，禁止覆盖同名 episode。

本工程后台只处理启动后新增的本工程样本，不接管 DAgger 或历史样本。
MCAP 上传同时复制引用的模型、标定和配置，验证共享资产后再发布 episode。
删除只针对符合原有分类、Home 和上传证明条件的本工程 episode；共享资产不删除。
MCAP 的远端上传回执放在父目录 `.upload-receipts/`，不添加到封口后的 episode 内。

## 离线检查与现场验收

下面命令应在具有 ROS Python 和 MCAP 依赖的采集环境中运行。
校验工具只读 episode，报告放在其外：

```bash
python3 -m teleop_data_collector.check_mcap \
  /path/to/A/episodeXX --out /path/to/new-report \
  --tools /home/descfly/.local/share/sharpa-conformance/recorder-conformance
```

软件格式样本（非训练数据、非真机认证）：

```bash
python3 -m teleop_data_collector.verify_mcap /path/to/new-software-fixture
```

吞吐回放（不创建 ROS 节点，不连接硬件，使用已有视频内容和软件纹理负载）：

```bash
python3 -m teleop_data_collector.benchmark_mcap \
  --source /path/to/data.mcap --out /path/to/new-benchmark \
  --seconds 10 --tactile --depth
```

现场验收仍需新采集正常回合，覆盖 RGB、头部深度、三路深度及三路深度加触觉，
对新输出运行 S1 检查。软件样本通过不能替代该步骤。
LeRobot 校验包用于后续实际转换出的训练包，不在此功能中修改转换器。
