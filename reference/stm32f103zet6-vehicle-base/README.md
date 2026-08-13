# STM32F103ZET6 车端复用基线

本目录保存一份可追溯的 STM32F103ZET6 车端源码快照，供团队比较和按需改造；它不是当前任何负责人的正式实现目录。

## 已迁移基线

- 源码快照：`firmware/stm32f103zet6/`
- 上游已提交版本：`b63f401799afd1d8a71458cbef7f02befb66343f`
- 上游提交说明：`release(car): archive flashed competition firmware`
- 保留范围：灰度循迹、TB6612/编码器、JY901、Pi 位姿入口、LoRa 协调、构建和烧录脚本，以及随源码分发的本地视觉代码。
- 排除项：ELF、ONNX、TensorRT engine、模型权重、YOLO 运行结果视频和其他可再生成/受管制工件未迁入；需要时按团队的模型发布流程单独取得。

## 使用方式

1. 先在目标负责人的目录建立实现与说明，再从本目录选择需要的文件。
2. 记录来源文件、目标硬件、接口差异和验证结果；不要把这里的引脚、速度、协议或现场结论直接搬到现车。
3. 仅在需要对照原始构建时，才可执行：

```powershell
Set-Location firmware/stm32f103zet6
cmake --preset LineFollowMissionDebug
cmake --build --preset LineFollowMissionDebug
```

构建成功只验证编译层。接线、传感器映射、轮向、编码器、无线时隙及实车表现需要在新目标上分别验证。

## 参考说明

- [`docs/control/README.md`](docs/control/README.md)：控制层级与状态。
- [`docs/control/line-follow-mission.md`](docs/control/line-follow-mission.md)：灰度循迹、速度和安全边界。
- [`docs/control/external-interfaces.md`](docs/control/external-interfaces.md)：Pi 位姿与 LoRa 的车端边界。
