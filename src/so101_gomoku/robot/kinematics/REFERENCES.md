# 模块参考与迁移边界

参考 lerobot-llm-chess/src/lerobot/model/kinematics.py、llm-tools/llm_toolkit.py。优先适配已有 IK；核对 URDF、关节方向、角度单位与限位。

上游源码统一保留在仓库根目录 references/，完整审阅说明见 references/README.md。当前移植状态与改造边界见 docs/architecture/IMPLEMENTATION_STATUS.md；原始源码仍保留在 references/。


