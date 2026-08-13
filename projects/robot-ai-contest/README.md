# 中国机器人及人工智能大赛小车项目

本轨道保存雷达小车历史中可追溯的 MSPM0 控制器快照及其时间线入口。它不是电赛 STM32 工程的分支，也不与电赛控制参数共享验证状态。

## 已迁移控制快照

| 项目 | 值 |
| --- | --- |
| 文件 | [`firmware/mspm0/phase9-airlink-ab-return-stop-20260710/empty.c`](firmware/mspm0/phase9-airlink-ab-return-stop-20260710/empty.c) |
| 归档分支 | `archive/phase9-airlink-ab-return-stop-20260710` |
| 归档提交 | `183e6ec947ee1a9d4bfcbf0137250e4adfd545df` |
| 调参标记 | `phase9_a_highleft_bx9_bsafe` |
| 归档记录 | A/B 码在飞机联调后到达目标、自动返航并在起点停车；记录为已实车测试并拍摄视频 |

该归档只包含 `empty.c` 与归档说明。缺少完整 MSPM0 工程、构建配置和依赖时，不得将其直接声明为可重建固件；应先补齐匹配工程并重新构建验证。

控制逻辑说明见 [`docs/control/README.md`](docs/control/README.md)，与电赛的关系见 [`../../docs/history/team-project-tracks.md`](../../docs/history/team-project-tracks.md)。

团队已提供经过多轮确认的 MSPM0G3507 通用引脚表；其活动组合、候选复用冲突与 SysConfig 验证流程见 [`docs/control/mspm0g3507-pin-assignment.md`](docs/control/mspm0g3507-pin-assignment.md)。
