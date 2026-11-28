# MCAP 实现离线验证记录（2026-11-28）

本记录对应 MCAP/rosbag 选择、可选深度/触觉、任务描述与 DAgger 兼容归档的实现。
验证进程没有连接设备、创建机器人控制发布者或操作正在运行的相机服务。
ROS 测试使用 `docker run --network none`；参考样本只读挂载。

## 已完成检查

| 检查 | 结果 |
|---|---|
| 数据采集包测试集 | 277 passed，227 skipped；跳过项主要为需配置只读转换 oracle 的差分测试及可选环境测试 |
| Qt 操作 GUI、统一 GUI、传输面板及新增设置测试 | 42 passed |
| 核心契约、主机传输、任务、设置、相机启动和 takeover 回归 | 69 passed；上传回执适配后的传输专项再次通过 25 项 |
| 新触觉数据契约 | 验证原生 SDK 时间、像素、F6、条件接触点、dtype/时钟拒绝，以及 SDK 读取仍在设备所有者线程 |
| RVL | 使用提供的独立解码器，验证含零值和随机 uint16 的大小端、带行填充输入逐像素往返 |
| MCAP 实际编码流水线 | 验证空闲样本不入档、H.264 完整回读、首末时间、CRC/索引/计数、时间倒退失败、资产引用与 SHA-256 |
| 控制器到归档 | 验证任务快照、停止、A/B 改评、切换格式后改评原 MCAP、D 类评级与容器校验结果分离 |
| 部署环境 | 已验证现有安装 overlay 可直接导入新模块，真实模型/标定与 74 路配置可加载 |
| 独立 recorder-conformance | 软件格式样本 27 项检查全部通过；深度和触觉的“不适用”项目不代表真机验收 |

NumPy 2 与现有 ROS OpenCV 扩展不兼容的问题已修正：采集依赖固定为 NumPy 1.26.4，
修正后旧 rosbag 转换回归通过。未修改转换器实现。

## DAgger 兼容对照

对照只读样本 `D/episode101/channel_stats.json`：本工程开启三路深度和触觉后，
74 个输出 topic 均存在于该 DAgger 样本，未增加不兼容的传感器/动作 topic。
DAgger 额外的 5 路为 ownership、dispatch_joints、control_context、dispatch、events；
它们属于 DAgger 执行上下文，本工程不补造这些记录。

归档保持标准消息、四路 action/command、原生状态、共享模型/标定/配置引用及轻量 episode。
本工程评级与生命周期扩展放在 episode.json，传输回执位于 episode 外。

## 吞吐测试

读取现有 DAgger 视频纹理，加入软件数值、原生大小的触觉、十指采样索引与三路深度负载。
不创建 ROS 节点，不打开 SDK。固定有界队列为 256，无覆盖、跳帧或扩大队列。

- 时长：10 秒。
- 接收：24,040 条；写入：24,040 条。
- 队列峰值：138/256。
- 最大生产端迟到约 3.9 ms。
- 含编码排空和封口的总耗时约 11.58 秒。

输出在工程忽略目录 `.runtime/validation/`：

- `benchmark-full-with-index/benchmark.json`：吞吐结果。
- `conformance-fixture-v2/report/report.md`：独立格式检查报告。
- `recording-settings.png`：离线 GUI 截图。

这些是软件测试材料，不能用作训练数据，也不构成新录制器的真机 S1 认证。

## 尚需现场完成

正常启动新配置后，分别采集 RGB、头部深度、三路深度、三路深度加触觉的正常回合，
对新输出运行 `teleop_data_collector.check_mcap` 完成 S1 检查。
本次未改变硬件使能、安全网关、限位、时效或速度保护策略。

相机配置副本按内容版本保存。已有共享相机服务仍维持原配置；需要在结束采集、
退出采集器后正常重启共享相机服务，不能只关闭再打开 GUI。
操作说明见 [MCAP_RECORDING.md](MCAP_RECORDING.md)。
