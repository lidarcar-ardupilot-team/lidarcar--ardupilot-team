# 黄总：无人机飞控

> 本目录维护无人机飞控、机载串口通信、飞行任务阶段及飞行验证。现已迁入一份 `ANO_LX` 源码快照：它是以 **STM32F407VGTx** 为目标的 Keil 工程，和根目录小车使用的 MSPM0G3507 是两套独立控制器，不能混用其引脚、构建工具或外设结论。

<details open>
<summary><strong>快速导航</strong></summary>

- [源码入口](firmware/ano_lx_stm32f407/ProjectSTM32F407/ANO_LX_STM32F407.uvprojx)：Keil 工程文件。
- [飞控核心代码](firmware/ano_lx_stm32f407/FcSrc/)：任务、控制、协议和 UART 逻辑。
- [板级与 MCU 驱动](firmware/ano_lx_stm32f407/DriversBsp/) / [STM32F407 驱动](firmware/ano_lx_stm32f407/DriversMcu/STM32F407/)。
- [阶段开发记录](firmware/ano_lx_stm32f407/该版本已完赛.txt)：随源码带入的原始文字记录，只作阶段回溯。
- [导入与验证边界](docs/飞控源码导入记录.md)：来源哈希、筛选规则和后续检查项。
- [板载视觉—飞控协议](../vision/docs/MaixCam与飞控串口协议.md)：与 MaixCam 的 UART3 交接，不是空地 AS32 帧。

</details>

## 当前内容与验证边界

| 内容 | 位置 | 状态 | 可以据此做什么 | 仍不能据此断言 |
| --- | --- | --- | --- | --- |
| ANO_LX 飞控源码 | [`firmware/ano_lx_stm32f407/`](firmware/ano_lx_stm32f407/) | `stage-record` | 阅读飞控任务、雷达/ESP/MaixCam 串口处理和工程结构。 | 当前机体可直接构建、可烧录、接线无误或已完成飞行。 |
| Keil 工程定义 | [`ProjectSTM32F407/ANO_LX_STM32F407.uvprojx`](firmware/ano_lx_stm32f407/ProjectSTM32F407/ANO_LX_STM32F407.uvprojx) | `stage-record` | 追溯目标芯片、源文件组和相对路径。 | 本地 Keil 版本、Pack 或生成产物仍与归档一致。 |
| 板载 ESP / 雷达 / MaixCam 串口逻辑 | [`FcSrc/my_uart.c`](firmware/ano_lx_stm32f407/FcSrc/my_uart.c) | `stage-record` | 对照帧解析、端口角色和软件状态。 | 当前线束、波特率、对端程序和故障策略已台架通过。 |
| 飞行验证 | `evidence/`（尚未迁入） | `planned` | 预留飞行记录落点。 | 任何源代码、截图或单段视频自动等于飞行验收。 |

状态标签的定义见 [项目资料与验证状态](../docs/项目资料与验证状态.md)。本次迁入保留可读源码和项目文件，不把压缩包里的历史构建产物带进当前 Git 历史。

## 工程结构

```text
firmware/ano_lx_stm32f407/
├── FcSrc/                         # ANO_LX、任务、PID、通信与控制逻辑
├── DriversBsp/                    # 板级初始化、光流/GPS 等 BSP
├── DriversMcu/STM32F407/          # MCU 驱动、CMSIS 与 ST 标准外设库
├── ProjectSTM32F407/
│   └── ANO_LX_STM32F407.uvprojx   # Keil 工程入口
└── 该版本已完赛.txt                # 原始阶段开发记录
```

工程文件声明的目标是 `STM32F407VGTx`、Cortex-M4、ARM-ADS 工具链。`ProjectSTM32F407/` 下的 `build/`、`.axf`、`.hex`、`.bin`、对象文件、J-Link 日志和用户 IDE 状态均未导入；这样源码树不会被某次本机编译结果污染。

## 接口分层：先分清三条链路

源码中的 `Drv_BSP.c` 初始化了 UART1、UART2、UART3 为 `115200`，并分别接入 ESP、雷达与 MaixCam 的接收处理。此处是**源码中的端口角色**，实际物理接线仍要以当前机体复核为准。

| 链路 | 源码落点 | 用途 | 不应混淆为 |
| --- | --- | --- | --- |
| 飞控 ↔ MaixCam | `my_uart.c`、`my_send_test.c`、`vision/src/main.py` | 飞控下发目标类别，视觉端回传机体系偏差。 | 空地 AS32 的 9 字节目标帧。 |
| 飞控 ↔ ESP | `my_uart.c`、`my_send_test.c` | 任务/二维码消息与对外目标帧的中继逻辑。 | 小车电机控制或视觉像素坐标。 |
| 飞控 ↔ 雷达 | `my_uart.c` | 接收定位/姿态类数据并更新飞控侧状态。 | 车端雷达协议。 |

空地 AS32 的应用层格式仍以 [空地联调通信约定](../docs/competition/空地联调通信约定.md) 为准；内部 MaixCam 帧的字节和坐标定义见 [板载视觉—飞控协议](../vision/docs/MaixCam与飞控串口协议.md)。

## 接手顺序

1. 先读 [导入记录](docs/飞控源码导入记录.md)，确认这是阶段源码快照而非当前机体的自动验收件。
2. 在只读/备份状态下打开 `.uvprojx`，核对目标芯片、Pack、源文件路径和编译输出位置；不要提交新生成的 `build/` 或 IDE 私有文件。
3. 对照当前机体逐项核对供电、端口、对端波特率和急停/任务状态，再记录一次最小台架结果。
4. 视觉、无线或任务阶段有改动时，先更新双方接口页，再获得同轮飞行证据；跨模块修改按 [协作与变更规则](../CONTRIBUTING.md) 进行。

小车路径规划和车端底盘安全不在本目录维护；视觉程序与其标定资料由 [vision/](../vision/) 维护。
