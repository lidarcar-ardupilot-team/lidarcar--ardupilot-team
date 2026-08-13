# MSPM0G3507 SysConfig PinMux 验证

## 结论

把团队引脚表录入 TI SysConfig 是确认“声明配置 → 生成配置”一致性的最有效步骤：SysConfig 会限制已选 UART 的可用 RX/TX，引脚复用冲突会显示为错误，并生成对应的配置代码。它**不能**替代 PCB 连通性、外设电平、波形和实际设备通信测试。TI 的 [MSPM0 SysConfig 指南](https://software-dl.ti.com/msp430/esd/MSPM0-SDK/latest/docs/english/tools/sysconfig_guide/doc_guide/doc_guide-srcs/sysconfig_guide.html) 明确说明 PinMux 会收窄可选资源并提示冲突；MSPM0G3507 有四个 UART，且不同封装的可用 GPIO 不同，须以 [TI 产品页/数据手册](https://www.ti.com/product/MSPM0G3507) 选择正确器件与封装。

## 建立配置

1. 新建或打开版本化的 `*.syscfg`，选择与 PCB 丝印一致的 **MSPM0G3507 精确封装**。
2. 添加 UART0–UART3、I2C、TIMA/TIMG、GPIO、ADC 与 SysTick/LED 所需模块；每个模块填写实际波特率、定时器周期、方向和中断需求。
3. 在每个模块的 **PinMux** 区选择本项目表中的唯一活动对；不得把同一候选同时分配给 UART、按键、LED、ADC、编码器或超声波。
4. 以 `mspm0g3507-pin-assignment.md` 的“已知互斥点”为核对清单，特别检查 PB0/PB1、PB4/PB5、PB6/PB7、PB12/PB13、PB15–PB18、PA24–PA31。
5. 使用 SysConfig 的 Problems/PinMux 摘要，要求 **0 个资源冲突和 0 个未解释警告**；保存 `.syscfg` 与生成的 `ti_msp_dl_config.c/.h`。

## 有效配置后的实物验证

| 子系统 | 最小验证 | 记录 |
| --- | --- | --- |
| UART | 外部回环或实际模块收发，核对波特率、TX/RX 方向和错误计数 | 端口、设备、配置、原始串口记录 |
| I2C 显示 | 地址扫描/显示初始化，测量 SDA/SCL 上拉与时序 | I2C 地址、波形或观测照片 |
| TIMG8/TB6612 | 抬轮逐电机确认 PWM、IN、STBY、方向与编码器符号 | 电机号、命令、轮向、计数 |
| TIMG6/TIMG7 | 手动转轮确认每侧计数和方向 | 左/右、计数变化、采样窗口 |
| ADC/循迹/超声波 | 单通道刺激与采样/边沿响应 | 物理位置、输入、极性、阈值 |

只有当 `.syscfg` 无冲突、生成代码已编译、信号在对应 PCB 引脚测得且外设完成预期收发时，才能将该引脚组合标记为 `bench-verified`。赛道或任务结论仍需单独的 `field-verified` 证据。
