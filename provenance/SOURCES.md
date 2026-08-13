# 迁移来源与完整性边界

## STM32 车端复用基线

| 字段 | 值 |
| --- | --- |
| 来源仓库 | `haimichenha/8-load-tracking-car` 的本地已提交工作树 |
| 来源目录 | `ZET6 HAl/lidar_car` |
| 提交 | `b63f401799afd1d8a71458cbef7f02befb66343f` |
| 提交说明 | `release(car): archive flashed competition firmware` |
| 迁入目录 | `reference/stm32f103zet6-vehicle-base/firmware/stm32f103zet6/` |
| 保留范围 | 已提交的构建配置、车端源码、脚本和随项目分发的视觉源码 |
| 排除范围 | 本地未提交内容、构建输出（含 ELF）、日志、AS 状态、CAD/媒体资产、模型权重/ONNX/TensorRT engine、YOLO 运行结果和运行凭据 |

## 雷达小车 MSPM0 控制快照

| 字段 | 值 |
| --- | --- |
| 来源仓库 | `haimichenha/all-search-stack` 历史归档 |
| 归档分支 | `archive/phase9-airlink-ab-return-stop-20260710` |
| 归档提交 | `183e6ec947ee1a9d4bfcbf0137250e4adfd545df` |
| 文件 Blob | `1f2a1c8b61f8b4abd530b4c001ffe501c906947a` |
| 迁入目录 | `car/firmware/mspm0/phase9-airlink-ab-return-stop-20260710/` |
| 完整性界限 | 仅包含 `empty.c`；不含匹配的完整工程与构建产物 |

## 团队提供的硬件资料

`car/docs/mspm0g3507-pin-assignment.md` 根据团队于本次迁移中提供并声明多轮确认的 MSPM0G3507 引脚表整理。该表的 PinMux 可行性由 TI 官方数据手册和 SysConfig 流程复核；具体 PCB 连通性仍以团队的实物证据为准。

### 雷达小车原理图导出

| 字段 | 值 |
| --- | --- |
| 本地来源 | `D:\Vs Code\MSPM0\lidarMSP\SCH_Schematic1_1_2026-08-13.pdf` |
| 迁入文件 | [`car/hardware/pcb/SCH_Schematic1_1_2026-08-13.pdf`](../car/hardware/pcb/SCH_Schematic1_1_2026-08-13.pdf) |
| 文件大小 | 324,096 bytes |
| SHA-256 | `70d55724bbee6695e336eb8b8b0eac7ca4393a7438a67219c6833823826c8f98` |
| 完整性界限 | 单页 PDF 导出；不包含原始 EDA 工程、Gerber 或 BOM。 |

### 联调视频证据索引

四段原始 MP4 保留在本地 `D:\Vs Code\MSPM0\lidarMSP\ppt creating\images\vedio联调\`，合计 1,165,821,367 bytes，未写入普通 Git 历史。可提交的联系表缩略图、原始视频哈希和用途说明见 [`car/evidence/20260710-airlink-ab-return/README.md`](../car/evidence/20260710-airlink-ab-return/README.md)。

### 压缩工作流

`car/docs/workflow.md` 的来源是 `D:\Vs Code\MSPM0\lidarMSP\skills\references\workflow.md`。迁入时只保留分层执行、日志证据、停止条件和 checkpoint 规则；原始调试过程以 `haimichenha/all-search-stack` 的历史分支与提交为追溯来源。

## 复核方法

对 STM32 复用基线，在来源仓库执行：

```powershell
git diff --no-index -- "ZET6 HAl/lidar_car" "<target>/reference/stm32f103zet6-vehicle-base/firmware/stm32f103zet6"
```

差异应仅来自本文件明确排除的路径。对 MSPM0 文件，以来源 Blob 和目标文件哈希复核。任何补齐的 PCB、树莓派或完整 MSPM0 工程都应增加新的来源条目，而不是覆盖本记录。
