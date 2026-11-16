# 上游参考项目与模块迁移索引

下载日期：2026-11-16。当前为 GitHub 官方 codeload 的 main 分支 ZIP 快照，不是 Git checkout；不支持直接 git pull。ZIP 原包保留用于追溯，摘要见 sources.json。未安装依赖、未运行上游代码、未连接机械臂。

## 本地源码

- [Koch_Gomoku_robot](Koch_Gomoku_robot/)：https://github.com/hri5-boardgame-robot/Koch_Gomoku_robot ，MIT。
- [lerobot-llm-chess](lerobot-llm-chess/)：https://github.com/BranavanSelvasingham/lerobot-llm-chess ，Apache-2.0。

原始源码整体保留，各业务目录用 REFERENCES.md 指向具体文件，避免拆散后破坏上游导入和资源路径。当前仅完成参考资料归档与迁移设计，尚未实现适配。

## 阅读结论

Koch 项目的 play.py 串联视觉、人类落子、MCTS 和实体动作；vision/initial.py、utils.py 包含手动透视校正、网格和圆检测；Gomoku_AI/Vanilla_MCTS 包含规则及神经网络 MCTS；omoku_bot_v2.py、robot.py 和 low_cost_robot/ 面向 Koch、Dynamixel 与 MuJoCo。适合参考闭环组织，不应把其电机驱动用于 SO101。

Koch 的 play.py 导入 VanillaBoard/VanillaGame/VanillaPolicyValueNet/VanillaMCTSPlayer，但构造时使用 Board/Game/PolicyValueNet/MCTSPlayer，存在明显名称不一致；不能按 README 推定开箱即用。视觉更新路径中还直接修改棋盘状态，迁移时须改为观测与已确认棋局分离。

SO101 项目主要参考 llm-tools/llm_toolkit.py、tool_move_piece.py、scripts/calibrate_board_transform.py 及 src/lerobot/model/kinematics.py。它已经将工具分发与 UI 分离，动作内部采用锁和多个抓放路径点。语言指令部分留作后续交互层参考。

SO101 的 perception/chess/piece_detector.py 仅做背景差分的 8×8 占用检测，不是黑白棋子分类器；board_pose_estimator.py 主要使用手选四角，并非现成的 ArUco 自动识别。board_model.py 使用棋格中心的半格偏移，五子棋应改为交点坐标；标定脚本对角点含义与棋盘宽度的约定也需统一核对。代码会寻找可能被 gitignore 排除的 URDF，不能假定源码包包含可用机器人模型。

## 迁移约定

1. 首期仍采用固定俯拍、9×9 自由五子棋、Alpha-Beta、固定供料。MCTS 与语言工具是扩展参考。
2. contracts 定义 BoardObservation（含 unknown、置信度）、GameState、Move、ExecutionResult，统一 row/col、单位和坐标系。
3. calibration 区分图像到棋盘单应映射与棋盘到机器人基座刚体变换；重新采集本机标定。
4. robot/primitives 将国际象棋“格到格移动”改成“固定供料位到交点放置”，接入已有 IK 与 SO101 驱动。
5. orchestration 独占执行控制；动作后视觉确认再提交棋局，异常暂停。
6. 复制或改编代码时保留对应版权、许可证及第三方声明；参考快照不作为默认运行依赖。

目前这两个项目已覆盖主要参考需求，暂不追加其他仓库。
