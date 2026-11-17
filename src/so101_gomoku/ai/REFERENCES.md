# 模块参考与迁移边界

参考 Koch_Gomoku_robot/Gomoku_AI/Vanilla_MCTS/mcts_alphaZero.py、policy_value_net.py 和 reference_model.py。此上游采用神经网络 MCTS；本项目首期 Alpha-Beta，需要独立实现或另行选择引擎。

上游源码统一保留在仓库根目录 references/，完整审阅说明见 references/README.md。当前移植状态与改造边界见 docs/architecture/IMPLEMENTATION_STATUS.md；原始源码仍保留在 references/。



## 当前实现（2026-11-17）
alpha_beta.py、candidates.py、evaluation.py 已实现本项目搜索。koch_mcts.py 实际导入vendor/koch网络与TreeNode并加载assets/models/koch/Vanilla_MCTS.model，不执行原play.py/VanillaBoard。支持在scripts/demos/play_gomoku.py中选择alphabeta或koch-mcts。改造细节与测试见docs/architecture/AI_AND_TURNS.md。
