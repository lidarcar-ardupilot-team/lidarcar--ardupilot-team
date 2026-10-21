# ANO_LX 飞控源码导入记录

> **状态：`stage-record`。** 本页固定本仓库中飞控源码的来源、筛选规则和可复核边界；它不代替一次新的构建、装机、台架或飞行验证。

## 本次导入

| 项目 | 记录 |
| --- | --- |
| 交付位置 | `D:\Vs Code\MSPM0\lidarMSP\提交\ANO_LX.zip` |
| 原始文件名 | `ANO_LX.zip` |
| SHA-256 | `76987DD94ACA650AF65EB04326F0DB7663547DAC44FC2740510EFE0F790F775C` |
| 压缩包根目录 | `ANO_LX/` |
| 当前仓库落点 | [`aircraft/firmware/ano_lx_stm32f407/`](../firmware/ano_lx_stm32f407/) |
| 工程入口 | `ProjectSTM32F407/ANO_LX_STM32F407.uvprojx` |
| 工程声明目标 | `STM32F407VGTx` / Cortex-M4 / ARM-ADS |

## 保留与排除

为保留源代码可读性和 Keil 工程的相对路径，本次保留 `FcSrc/`、`DriversBsp/`、`DriversMcu/STM32F407/`、工程 `.uvprojx`、调试配置、库源码和原始阶段记录 `该版本已完赛.txt`。

以下内容没有迁入：嵌套 `.git/` 历史、`.vscode/`、`ProjectSTM32F407/build/`、`.axf`/`.hex`/`.bin`、对象/依赖/列表文件、J-Link 日志、Keil 用户状态和锁文件。这些文件要么可由构建再生，要么只反映某一台电脑的状态；对应忽略规则写在仓库根目录 [`.gitignore`](../../.gitignore)。

原压缩包的 Keil 工程还引用了 `..\\Doc\\note.txt`，但该文件不在压缩包中，因此本次没有虚构补件。打开工程时若该条目提示缺失，应先确认它在工程中的用途和原始来源，再单独补入并记录来源。

## 复核清单

将这份快照用于现役机体前，至少补齐下列记录：

```text
当前机体/飞控板版本：
Keil、Pack 与编译结果：
UART1/2/3 的实际对端、TX/RX 方向与波特率：
视觉、ESP、雷达程序版本：
任务/解锁/急停的台架检查：
飞行场地、操作人、通过判据与原始证据：
```

源码里出现的端口、任务和坐标常量可以作为复查起点；未写进同轮记录的接线、校准和飞行表现仍保持未知。
