# 模块参考与迁移边界

参考 lerobot-llm-chess/llm-tools/tool_move_piece.py、tool_open_gripper.py、tool_close_gripper.py；Koch 的 omoku_bot_v2.py。改为固定供料到交点，增加结果反馈与视觉验证。

上游源码统一保留在仓库根目录 references/，完整审阅说明见 references/README.md。当前移植状态与改造边界见 docs/architecture/IMPLEMENTATION_STATUS.md；原始源码仍保留在 references/。


