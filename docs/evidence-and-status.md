# 资料与证据索引

> 此页帮助阅读者判断“已有材料是什么”与“它能证明什么”。它不把历史快照、图片或单次视频包装成当前系统的整体验收。

## 状态标签

| 标签 | 含义 | 最低支撑 |
| --- | --- | --- |
| `planned` | 已提出但未验证 | 需求或待办；不能宣称已可用。 |
| `team-confirmed` | 团队多轮核对过的约定/分配 | 记录来源；涉及硬件时仍要与实际 PCB 和量测核对。 |
| `bench-verified` | 台架、单模块或最小链路通过 | 固件/配置版本、接线、操作、观察/日志。 |
| `field-verified` | 在记录的实际场地/任务条件下验证 | 同轮版本、环境、完整证据与通过判据。 |
| `reference` | 历史快照、旧工程或外部对照 | 来源、范围和差异；不能直接继承为当前结果。 |

## 已归档材料

| 材料 | 位置 | 当前标签 | 能支持的范围 | 仍不能支持 |
| --- | --- | --- | --- | --- |
| MSPM0 `empty.c` 控制快照 | [car/firmware/mspm0/](../car/firmware/mspm0/) | `reference` | 历史 A/B 路线、返航、停止相关逻辑的来源可追溯 | 当前完整工程可构建、新板 PinMux 或新赛道结论。 |
| MSPM0G3507 引脚分配 | [car/docs/mspm0g3507-pin-assignment.md](../car/docs/mspm0g3507-pin-assignment.md) | `team-confirmed` | 团队当前角色与互斥关系的核对入口 | PCB 连通、所选封装可用性和外设实物通信。 |
| SysConfig 验证流程 | [car/docs/mspm0g3507-sysconfig-validation.md](../car/docs/mspm0g3507-sysconfig-validation.md) | 方法文档 | 生成配置与台架核对步骤 | 任何未执行配置的结果。 |
| 雷达车原理图 PDF | [car/hardware/](../car/hardware/) | 归档设计资料 | 导出时的连线/接口设计信息 | 原始 EDA、Gerber、BOM、实物焊接和板测。 |
| A/B 历史联调视频索引 | [car/evidence/20260710-airlink-ab-return/](../car/evidence/20260710-airlink-ab-return/) | `reference` | 原始视频名称、大小、哈希和预览可追溯 | 当前固件、PCB、场地或重复性。 |
| STM32F103ZET6 车端基线 | [reference/stm32f103zet6-vehicle-base/](../reference/stm32f103zet6-vehicle-base/) | `reference` | 被迁入源码的来源、构建和比较入口 | MSPM0 或本项目任一当前模块的引脚/协议/验收。 |
| 无人机与视觉正式材料 | [aircraft/](../aircraft/) / [vision/](../vision/) | `planned` | 职责入口 | 飞行、识别、投递或跨端接口已经完成。 |

## 新证据最小登记

新增一份照片、视频、日志、PCB 或测试结论时，至少写明：

```text
日期与地点：
所属模块和负责人：
硬件/PCB 与固件/模型版本：
操作与通过判据：
原始证据路径、哈希或发布链接：
观测结果与状态标签：
不覆盖的条件/限制：
```

原始大视频、模型权重和可能含敏感信息的日志不直接放入普通 Git 历史；用 GitHub Release、Git LFS 或经授权的存储发布，并在索引中留文件名、哈希与稳定链接。
