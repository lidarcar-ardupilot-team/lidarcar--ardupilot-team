# 韦少：视觉识别与舵机投递

本目录预留给韦少维护视觉识别、模型/数据发布、目标结果接口、舵机投递逻辑和相应验证记录。

可参考的已有材料位于：

- 视觉源码：[`../reference/stm32f103zet6-vehicle-base/firmware/stm32f103zet6/yolov5/`](../reference/stm32f103zet6-vehicle-base/firmware/stm32f103zet6/yolov5/)
- 舵机测试入口：[`../reference/stm32f103zet6-vehicle-base/firmware/stm32f103zet6/Hardware/app_servo_test.c`](../reference/stm32f103zet6-vehicle-base/firmware/stm32f103zet6/Hardware/app_servo_test.c)

这些材料仅供改造参考，模型权重和推理工件未随迁移提交。正式实现应在本目录新增 `firmware/`、`docs/` 与 `evidence/`，并明确相机/识别结果、投递触发条件、舵机行程、安全位置和实测证据。无人机飞行控制与雷达小车规划不在本目录维护。
