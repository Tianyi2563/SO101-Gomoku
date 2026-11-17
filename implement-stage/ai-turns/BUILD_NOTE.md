# Build ladder
F0 MUST 入口→标记模拟stub→smoke JSON，先运行。
F1 MUST Alpha-Beta与规则历史，测试合法/攻防/超时/终局。
F2 MUST 上游NN+MCTS真实权重推理，测试编码/合法性/边界。
F3 MUST 回合状态机、验证提交、暂停恢复，测试故障路径。
F4 MUST 两引擎多回合demo+累计测试+审阅。

## Run record
## Deferred
真实视觉/运动与标定，AI棋力评估和重启恢复。
## Notes
保留工作区改动供用户审阅，不自动提交或推送。

## Run record
F0: smoke入口exit0，标记模拟占位。
F1: Alpha-Beta6项测试exit0。
F2: 真实torch权重加载+网络/MCTS3项测试exit0（累计AI9项）。
F3: controller10项后扩展14项exit0。
F4: 累计45项exit0；Alpha-Beta5轮10步、MCTS2轮4步均完成，棋局与模拟物理盘一致。额外终端交互、机器人先手和注入失败示例均运行。
发现Windows快速time.time重复导致首轮demo正确暂停；修复模拟时钟而未放宽生产观测规则。T12在修复前声明。初次测试41项后，最终45项通过。
产物与复现命令见docs/architecture/AI_AND_TURNS.md。

## Final
47项累计测试exit0，AB5轮/MCTS2轮在缓存帧修复后重跑exit0；额外整局自动对弈正常终局。审阅第一轮发现1个缓存观测缺陷和1条陈旧ledger描述，已修复并加T14；第二轮聚焦复核clean且独立重跑16项controller测试通过，同家族provisional。没有未解决审阅发现。
F0-F4均通过其软件检查；无存活的未标记功能占位，simulation明确保持合成硬件/视觉。真实硬件、AI棋力评估与持久化恢复继续deferred。仍未自动提交/推送。
