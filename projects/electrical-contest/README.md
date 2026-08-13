# 电赛小车项目

本轨道保存电赛 STM32F103ZET6 车端代码与控制文档。它与机器人赛共用团队和 PCB 管理，但不继承机器人赛的路线、雷达参数或现场结论。

## 已迁移基线

- 源码快照：`firmware/stm32f103zet6/`
- 上游已提交版本：`b63f401799afd1d8a71458cbef7f02befb66343f`
- 上游提交说明：`release(car): archive flashed competition firmware`
- 迁移范围：灰度循迹、TB6612/编码器、JY901、Pi 位姿入口、LoRa 协调、构建和烧录脚本，以及随源码分发的本地视觉代码。
- 排除项：ELF、ONNX、TensorRT engine、模型权重、YOLO 运行结果视频和其他可再生成/受管制工件未迁入；需要时按团队的模型发布流程单独取得。

该提交是可追溯源码基线；重新构建、烧录或实车运行前仍须以本项目文档和当前硬件复核为准。

## 开发入口

```powershell
Set-Location firmware/stm32f103zet6
cmake --preset LineFollowMissionDebug
cmake --build --preset LineFollowMissionDebug
```

构建成功只验证编译层。接线、传感器映射、轮向、编码器、无线时隙及赛道表现需分别测试。

## 控制文档

- [`docs/control/README.md`](docs/control/README.md)：控制层级与状态。
- [`docs/control/line-follow-mission.md`](docs/control/line-follow-mission.md)：灰度循迹、速度和安全边界。
- [`docs/control/external-interfaces.md`](docs/control/external-interfaces.md)：Pi 位姿与 LoRa 的车端边界。
