# LidarCar Team

本仓库服务于同一支小车团队的协作与交接。电赛与中国机器人及人工智能大赛共用团队成员、PCB 与基础车辆平台，但其赛题、控制策略、验证记录和调试时间线彼此独立。

## 先读这里

- 共用搭建与上电：[`shared/vehicle-build/`](shared/vehicle-build/)
- PCB 资料登记：[`shared/pcb/`](shared/pcb/)
- 电赛控制与源码：[`projects/electrical-contest/`](projects/electrical-contest/)
- 中国机器人及人工智能大赛控制快照：[`projects/robot-ai-contest/`](projects/robot-ai-contest/)
- 两条独立时间线：[`docs/history/team-project-tracks.md`](docs/history/team-project-tracks.md)
- 迁移来源与边界：[`provenance/SOURCES.md`](provenance/SOURCES.md)

## 协作原则

1. `shared/` 只放两项赛事确实共用的硬件事实、PCB 版本和通用上电流程。
2. 赛事专用代码、控制参数、现场结论和日志只放在各自的 `projects/<event>/` 中。
3. 任何验证结论必须标注为 `planned`、`bench-verified`、`field-verified` 或 `reference`；构建通过不等于实车通过。
4. 构建目录、烧录产物、日志、模型权重、凭据和 IDE 私有状态不得提交。

## 当前迁移内容

- 电赛：STM32F103ZET6 车端的已提交源码快照，包含灰度循迹、TB6612 驱动、编码器、JY901、Pi 位姿入口与 LoRa 协调代码。
- 机器人赛：保留最后一次 A/B 飞机联调返航停车的 MSPM0 `empty.c` 控制器快照。它是单文件归档，不被声明为完整可构建工程。

新增或修改硬件、引脚、控制策略前，先更新对应项目文档与证据记录，再提交代码。
