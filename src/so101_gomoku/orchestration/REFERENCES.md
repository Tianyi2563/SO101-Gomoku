# 模块参考与迁移边界

参考 Koch_Gomoku_robot/play.py、interface.py 和 SO101 llm-tools/llm_toolkit.py 的执行锁。重构回合状态机，失败暂停，验证后提交棋局。

上游源码统一保留在仓库根目录 references/，完整审阅说明见 references/README.md。当前移植状态与改造边界见 docs/architecture/IMPLEMENTATION_STATUS.md；原始源码仍保留在 references/。


