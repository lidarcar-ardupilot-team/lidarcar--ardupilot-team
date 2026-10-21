# MaixCam Pro 与 STM32F407 飞控：板载串口协议

> **状态：`stage-record`。** 本页由已迁入的 `vision/src/main.py` 与 `aircraft/firmware/ano_lx_stm32f407/FcSrc/my_uart.c`、`my_send_test.c` 对照整理，描述该源码快照的板载链路；没有替代接线核对或台架联调。

<details open>
<summary><strong>目录</strong></summary>

- [协议边界](#boundary)
- [飞控到相机：选择目标](#fc-to-camera)
- [相机到飞控：回传偏差](#camera-to-fc)
- [坐标与控制权](#coordinates)
- [源码交叉索引与联调清单](#checklist)

</details>

<a id="boundary"></a>
## 协议边界

这条链路位于**无人机机载内部**：STM32F407 飞控与 MaixCam Pro 通过 UART 交接目标类别和视觉偏差。它使用 `0xBB` 作为帧头，**不等同于**无人机经 ESP/AS32 发给小车、参赛电脑的 `AA BB target_id qr_id x_hi x_lo y_hi y_lo FF` 9 字节帧。对外帧见 [空地联调通信约定](../../docs/competition/空地联调通信约定.md)。

导入代码双方均配置为 `115200`：飞控端 `Drv_BSP.c` 初始化 UART3；视觉端配置 `/dev/ttyS1`。实际端口、引脚、电平与 TX/RX 方向必须以当前机体台架记录为准。

<a id="fc-to-camera"></a>
## 飞控 → 相机：选择目标

```text
BB code FF
```

| 字节 | 含义 |
| --- | --- |
| `BB` | 帧头。 |
| `code` | 目标类别：A=`CA`、B=`CB`、C=`CC`。 |
| `FF` | 帧尾。 |

飞控源码 `my_send_maixcam()` 经 UART3 发送该 3 字节帧；视觉端 `parse_fc_cmd()` 只接受 A/B/C 三个有效类别。收到新目标时，视觉端清空滤波、投放保持和舵机状态，然后仅在当前帧的标签中选择相应类别且面积最大的目标。

<a id="camera-to-fc"></a>
## 相机 → 飞控：回传视觉偏差

```text
BB code ctrl x_sign x_hi x_lo y_sign y_hi y_lo FF
```

总长 10 字节。视觉端使用“符号位 + 绝对值 uint16”的编码，飞控端在 `MY_uart_maixcam_anl()` 中按相同规则还原为有符号 `int16` 厘米值。

| 字段 | 含义 |
| --- | --- |
| `BB` / `FF` | 帧头 / 帧尾。 |
| `code` | A=`CA`、B=`CB`、C=`CC`；飞控源码另保留巡线类别 `A1`。 |
| `ctrl` | `50`：交还飞控控制；`60`：视觉引导；飞控源码另保留巡线控制 `61`。 |
| `x_sign x_hi x_lo` | X 的负号位、绝对值高字节、绝对值低字节。`x_sign=01` 表示负。 |
| `y_sign y_hi y_lo` | Y 的负号位、绝对值高字节、绝对值低字节。`y_sign=01` 表示负。 |

无目标时，视觉端每 5 个循环发送一次 `ctrl=50, x=0, y=0` 的释放帧；有有效目标时发送 `ctrl=60` 和滤波后的偏差。飞控接收端还会检查帧长、帧头尾、类别/控制权匹配和两处符号位合法性。

<a id="coordinates"></a>
## 坐标与控制权

`main.py` 的图像坐标为 `dx` 向右为正、`dy` 向下为正；默认映射为：

```text
body_x = -dy_ground    # 机头前方为正
body_y = -dx_ground    # 机头左侧为正
```

其中 `cm_per_pixel = CAMERA_HEIGHT_CM / FOCUS_LEN_PX`。安装反向、相机偏航和比例修正由 `BODY_*_SIGN`、`CAMERA_YAW_DEG`、`GAIN_*` 等配置承担；这些只是源码快照中的可调位置，不能把默认数值写成通用场地坐标。

视觉端在软件层面仅产生偏差和投放触发条件。飞行阶段、解锁、故障处理与实际执行权仍由飞控维护；与小车相关的目标坐标还需经过空地通信和地图坐标转换。

<a id="checklist"></a>
## 源码交叉索引与联调清单

| 内容 | 源码位置 |
| --- | --- |
| MaixCam 初始化、标签识别、坐标/舵机状态机 | [`vision/src/main.py`](../src/main.py) |
| 飞控 UART3 接收和 10 字节解析 | [`aircraft/FcSrc/my_uart.c`](../../aircraft/firmware/ano_lx_stm32f407/FcSrc/my_uart.c) |
| 飞控下发 3 字节选择帧 | [`aircraft/FcSrc/my_send_test.c`](../../aircraft/firmware/ano_lx_stm32f407/FcSrc/my_send_test.c) |
| 飞控端 UART 角色绑定 | [`aircraft/Drivers/.../Drv_Uart.c`](../../aircraft/firmware/ano_lx_stm32f407/DriversMcu/STM32F407/Drivers/Drv_Uart.c) |

台架记录至少应保存：双方提交哈希、实际 UART 对端/引脚/电平、115200 配置、A/B/C 三种选择帧、正负 X/Y 回传样例、丢失目标后的 `ctrl=50` 行为、舵机机械零位与异常回退。完成这些前，本页只能作为读码和核对入口。
