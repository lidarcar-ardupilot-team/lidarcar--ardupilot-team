# 协作与变更规则

> 新成员先读仓库根目录的“快速导航”，跨模块工作再读 [`docs/team-collaboration.md`](docs/team-collaboration.md)。本文只规定每次修改的最小提交边界。

## 变更落点

- 修改某模块的 PCB、供电、车架或接线：同时更新该负责人目录下对应的硬件 Markdown。
- haimichenha 的雷达小车、黄总的无人机、韦少的视觉与舵机投递：只更新各自的 `car/`、`aircraft/`、`vision/` 目录及其说明。
- 从阶段文件或后续引入的第三方项目复用代码：在目标目录记录目标文件、MSPM0G3507/硬件接口差异和验证结果；不能直接把阶段资料当作当前实车结论。
- 修改跨模块消息、坐标或触发条件：先补充双方可读的接口记录，再改代码；修改硬件时，同步更新实际使用该硬件的负责人资料。

## 每次控制变更的最小记录

在负责人目录中记录：输入来源、输出执行器、参数或状态机变化、验证命令/日志、验证等级与回退提交。一次实车成功只能说明所记录的条件下通过，不能覆盖其他职责或其他硬件配置。

状态标签使用 [`docs/evidence-and-status.md`](docs/evidence-and-status.md) 的 `planned`、`team-confirmed`、`bench-verified`、`field-verified` 或 `stage-record`；不要把“构建成功”“看起来能动”写成 `field-verified`。

## 提交前检查

```powershell
git diff --check
git status --short
```

确认没有构建产物、模型权重、私钥、网络凭据、运行日志或 IDE 私有状态后再提交。
