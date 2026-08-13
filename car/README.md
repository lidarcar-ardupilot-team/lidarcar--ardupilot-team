# 陈海：雷达小车

本目录由陈海维护，保存雷达小车的 MSPM0 控制、引脚分配、雷达/编码器/姿态输入、路径规划、返航与电机驱动相关说明和证据。

## 当前内容

- 历史控制器快照：[`firmware/mspm0/phase9-airlink-ab-return-stop-20260710/empty.c`](firmware/mspm0/phase9-airlink-ab-return-stop-20260710/empty.c)。这是来源可追溯的单文件归档，尚不是完整可构建工程。
- 控制说明：[`docs/README.md`](docs/README.md)
- MSPM0G3507 引脚表：[`docs/mspm0g3507-pin-assignment.md`](docs/mspm0g3507-pin-assignment.md)
- SysConfig 验证流程：[`docs/mspm0g3507-sysconfig-validation.md`](docs/mspm0g3507-sysconfig-validation.md)

## 职责边界

陈海负责小车本体的定位、规划、底盘驱动、雷达链路和相关集成记录；不在此目录维护无人机飞行控制，也不维护视觉识别模型与舵机投递逻辑。跨职责接口应由双方在各自目录同步记录。

本目录下的 Markdown 可直接作为雷达小车说明文件修改。新增完整工程时，应保留版本化 `*.syscfg`、生成配置、构建方式、PCB 修订号和台架/实车证据。
