# 雷达小车硬件资料

## 原理图导出

| 项目 | 值 |
| --- | --- |
| 文件 | [`pcb/SCH_Schematic1_1_2026-08-13.pdf`](pcb/SCH_Schematic1_1_2026-08-13.pdf) |
| 导出日期 | 2026-08-13（以文件名为准） |
| 图纸信息 | 单页、A4、雷达车 1.0 |
| SHA-256 | `70d55724bbee6695e336eb8b8b0eac7ca4393a7438a67219c6833823826c8f98` |
| 来源 | haimichenha 本地 `lidarMSP` 工作目录的原始导出文件 |

该 PDF 以原始字节迁入，用于核对实际连接、接口和板级电源关系；它不是原始 EDA 工程，也不自动证明实物焊接、外设通信或引脚复用已经通过测试。

## 使用与核对顺序

1. 以实物 PCB、线束和量测结果确认连接；任何差异先记录，不凭旧代码猜测。
2. 在版本化 `*.syscfg` 中选择实际器件与封装，消除 PinMux 资源冲突并保存生成配置。
3. 对照 [`../docs/mspm0g3507-pin-assignment.md`](../docs/mspm0g3507-pin-assignment.md) 更新当前角色分配；发生冲突时同步修改引脚表、`*.syscfg` 和固件。
4. 对 UART、I2C、编码器、电机、ADC 和超声波分别完成台架记录后，才可标记为 `bench-verified`。

原始 EDA 工程、Gerber、BOM、板号照片和板测记录仍待补充；其导入规则见 [`../../shared/pcb/README.md`](../../shared/pcb/README.md)。
