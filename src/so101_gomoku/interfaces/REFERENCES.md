# 模块参考与迁移边界

参考 lerobot-llm-chess/chess_robot_ui_llm_v2.py 与 llm-tools/tool_*.py。后续接入 JARVIS；语言请求先经过规则及执行状态检查，再进入机器人任务。

上游源码统一保留在仓库根目录 references/，完整审阅说明见 references/README.md。当前移植状态与改造边界见 docs/architecture/IMPLEMENTATION_STATUS.md；原始源码仍保留在 references/。


