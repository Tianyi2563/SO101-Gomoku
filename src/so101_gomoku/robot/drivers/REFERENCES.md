# 模块参考与迁移边界

参考 lerobot-llm-chess/src/lerobot/robots/so101_follower/、motors/feetech/。Koch 的 robot.py、dynamixel.py 属于其他硬件，不直接复用。

上游源码统一保留在仓库根目录 references/，完整审阅说明见 references/README.md。当前移植状态与改造边界见 docs/architecture/IMPLEMENTATION_STATUS.md；原始源码仍保留在 references/。


