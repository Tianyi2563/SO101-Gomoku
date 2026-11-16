# 模块参考与迁移边界

统一观测、棋局、棋步和执行结果；参考 Koch 的 Gomoku_AI/Vanilla_MCTS/game.py 与 SO101 的 src/lerobot/perception/chess/board_model.py。不能直接沿用两套不同棋盘编码。

上游源码统一保留在仓库根目录 references/，完整审阅说明见 references/README.md。当前移植状态与改造边界见 docs/architecture/IMPLEMENTATION_STATUS.md；原始源码仍保留在 references/。


