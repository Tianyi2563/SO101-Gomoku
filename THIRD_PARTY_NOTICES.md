# 第三方代码来源

完整许可证保存在 docs/licenses/。references/ 保留下载的源码与归档记录。

| 业务文件 | 来源与处理 | 许可证 |
|---|---|---|
| robot/drivers/vendor/lerobot_so101/so101_follower.py、config_so101_follower.py | lerobot-llm-chess 的 src/lerobot/robots/so101_follower/ 原样复制，仅作驱动参考，不运行其相对导入 | Apache-2.0，原版权头保留 |
| robot/kinematics/placo_kinematics.py | lerobot-llm-chess 的 src/lerobot/model/kinematics.py 原样复制，可选求解器；需另装 placo 和提供本机 URDF | Apache-2.0，原版权头保留 |
| calibration/board_geometry.py | 改编 SO101 的 perception/chess/board_model.py 思路，去除 chess/SE3/draccus 依赖，改成交点并校验参数 | Apache-2.0 |
| vision/geometry.py | 改编 Koch 的 vision/utils.py 的网格与透视函数；移除隐式缩放并加校验 | MIT，Copyright (c) 2024 hri5-boardgame-robot |
| game/board.py | 改编 Koch 的 Gomoku_AI/Vanilla_MCTS/game.py 四方向连五扫描，重新实现不可变棋盘与合法性验证 | MIT，Copyright (c) 2024 hri5-boardgame-robot |
| robot/planning/pick_place_plan.py、robot/primitives/pick_and_place.py | 改编 SO101 的 llm-tools/tool_move_piece.py 路径顺序，独立规划、执行、失败传播与抓取验证 | Apache-2.0 |

所有业务代码相对上游的变更日期：2026-11-16。其余新增模块为本项目接口与适配实现。上游作者及贡献者信息以各原项目与保留的版权头为准。未为整个项目另行选择许可证。

## SO101 官方模型（2026-11-16）
assets/robot/so101/ 中 URDF、STL 资源与 README 来自 TheRobotStudio/SO-ARM100/Simulation/SO101，官方原样快照，Apache-2.0（同目录 LICENSE）。非此前两个参考项目的复制品。验证工具为本项目新增。
