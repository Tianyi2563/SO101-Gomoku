# 模块参考与迁移边界

参考 lerobot-llm-chess/scripts/calibrate_board_transform.py、src/lerobot/perception/chess/board_model.py。重定义 9×9 交点原点，去掉棋格中心半格偏移，使用独立点验证误差。

上游源码统一保留在仓库根目录 references/，完整审阅说明见 references/README.md。当前移植状态与改造边界见 docs/architecture/IMPLEMENTATION_STATUS.md；原始源码仍保留在 references/。


