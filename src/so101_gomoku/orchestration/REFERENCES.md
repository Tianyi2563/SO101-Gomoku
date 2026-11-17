# 模块参考与迁移边界

参考 Koch_Gomoku_robot/play.py、interface.py 和 SO101 llm-tools/llm_toolkit.py 的执行锁。重构回合状态机，失败暂停，验证后提交棋局。

上游源码统一保留在仓库根目录 references/，完整审阅说明见 references/README.md。当前移植状态与改造边界见 docs/architecture/IMPLEMENTATION_STATUS.md；原始源码仍保留在 references/。



## 当前实现（2026-11-17）
controller.py实现完整同步回合与显式恢复，executor.py桥接planner和PickAndPlace，simulation.py供离线测试。实际视觉/标定仍需后续接入。边界与测试见docs/architecture/AI_AND_TURNS.md。
