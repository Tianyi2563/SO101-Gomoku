# 模块参考与迁移边界

参考 Koch_Gomoku_robot/vision/initial.py、vision/utils.py；SO101 的 src/lerobot/perception/chess/board_pose_estimator.py、piece_detector.py。补充 ArUco、黑白/空/unknown 分类和多帧稳定检测。

上游源码统一保留在仓库根目录 references/，完整审阅说明见 references/README.md。当前移植状态与改造边界见 docs/architecture/IMPLEMENTATION_STATUS.md；原始源码仍保留在 references/。


