# haimichenha：雷达小车

> 本目录是雷达小车的职责入口，维护 MSPM0 控制、雷达/编码器/姿态输入、路径规划、返航、底盘驱动及其证据。它不维护无人机飞控，也不维护视觉模型或投递逻辑。

## 先看状态

| 项目 | 当前状态 | 阅读边界 |
| --- | --- | --- |
| MSPM0 控制器 | `stage-record` | [`empty.c` 阶段控制文件](firmware/mspm0/phase9-airlink-ab-return-stop-20260710/empty.c) 便于复盘，但不是完整可构建工程。 |
| 引脚与 PinMux | `team-confirmed` → 待台架复核 | 先读 [引脚分配](docs/mspm0g3507-pin-assignment.md) 与 [SysConfig 验证](docs/mspm0g3507-sysconfig-validation.md)。 |
| 硬件资料 | 部分归档 | 有 [原理图 PDF 与核对顺序](hardware/README.md)；EDA 源、Gerber、BOM 和板测待补。 |
| A/B 联调记录 | `stage-record` | 有 [视频预览/哈希索引](evidence/20260710-airlink-ab-return/README.md)，不自动覆盖新板、新固件或新场地。 |

## 按任务进入

- **恢复或新建 MSPM0 工程：** [控制资料入口](docs/README.md) → [开发工作流](docs/workflow.md)。先补 `.syscfg`、生成配置、构建方式和硬件修订号。
- **核对 PCB、引脚和外设：** [硬件资料](hardware/README.md) → [引脚分配](docs/mspm0g3507-pin-assignment.md)。物理连通与 SysConfig 冲突未消除前，不用旧代码反推接线。
- **复盘历史 A/B 联调：** [视频证据索引](evidence/20260710-airlink-ab-return/README.md)。它只描述归档条件下的材料。
- **核对当前控制器与引脚：** 只以 [MSPM0G3507 引脚分配](docs/mspm0g3507-pin-assignment.md)、版本化 `.syscfg`、PCB 和实物量测为准。
- **与无人机/视觉联调：** 先建立 [跨模块接口记录](../docs/team-collaboration.md#interface-record)，再进入串联测试。

## 提交要求

每次控制、引脚、路线或停车行为变化，至少记录：输入来源、输出执行器、参数/状态机差异、PCB/固件版本、最小验证命令、原始证据与回退提交。状态定义和材料边界见 [资料与证据索引](../docs/evidence-and-status.md)。
