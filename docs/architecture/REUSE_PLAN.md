# 代码复用与首期实现设计

状态：设计稿，2026-11-16。依据本地 references/ 两份源码的静态审阅；尚未安装依赖或做软件、硬件运行验证。直接复用表示业务算法基本无需改写，仍需验证本机依赖、配置和接口。首期假设：单臂、固定俯拍、9×9 自由五子棋、固定供料、Alpha-Beta；几何尺寸与硬件参数待实测。

## 一、总体取舍

沿用 SO101 的硬件基础能力；改造两项目的几何、标定与抓放流程；自行设计本项目的观测、棋局状态、回合状态机与验证闭环。保持 references 原样，通过业务模块适配器接入，不将上游目录直接加入全局导入路径。

## 二、逐模块复用表

| 我们的目录 | 分类 | 上游依据 | 首期处理 |
|---|---|---|---|
| robot/drivers | 底层直接复用，适配层自写 | SO101: src/lerobot/robots/so101_follower/、motors/feetech/ | 使用选定、固定版本的 LeRobot 连接、读关节、发送目标及夹爪控制；封装本项目 RobotDriver。现有 fork 与所选版本兼容性需验证，不单独拷贝一个驱动文件 |
| vision 的相机采集 | 直接使用已有能力 | SO101: src/lerobot/cameras/opencv/ | 选用 LeRobot 相机封装或 OpenCV VideoCapture 之一，避免两套代码同时占用相机 |
| robot/kinematics | 修改后复用，优先已有 IK | SO101: src/lerobot/model/kinematics.py | 封装已有求解器；如不满足再接 placo。统一关节顺序、度/弧度、末端坐标系，补目标误差及关节限位检查 |
| calibration | 修改后复用 | SO101: scripts/calibrate_board_transform.py、perception/chess/board_model.py | 保留示教点和刚体坐标框架；改为交点模型，统一角点含义，重新采集本机数据，输出未参与拟合点的误差 |
| vision 的透视与网格 | 修改后复用 | Koch: vision/initial.py、utils.py 的 manual_warping、warp_planar、get_grid_points | 保留手选四角作为调试入口；新增 ArUco 自动定位，返回完整交点坐标与有效性 |
| vision 的棋子识别 | 重新设计，局部借鉴 | Koch: update_board_circle；SO101: piece_detector.py | 逐交点 ROI 判断 empty/black/white/unknown，带置信度与时间戳；全盘观测、多帧稳定、遮挡拒绝更新；圆检测只作辅助 |
| game | 重新实现精简规则，借鉴胜负扫描 | Koch: Gomoku_AI/Vanilla_MCTS/game.py | 自有统一棋盘编码、轮次、合法落子、四方向连五与满盘判断；去掉禁手、神经网络状态和硬件限制耦合 |
| ai | 首期自行实现 | Koch: mcts_alphaZero.py、policy_value_net.py 作为后续方案 | 候选点生成、攻防评分、必胜/必防、限时迭代加深 Alpha-Beta；超时时返回已有合法候选 |
| robot/planning | 重新设计，借鉴路径顺序 | SO101: llm-tools/tool_move_piece.py | 根据供料位、目标交点、实测高度生成路径点；检查 IK、限位、工作区和密集棋子间隙，不能把几个高位路径点当作碰撞保证 |
| robot/primitives | 修改后复用 | SO101: tool_move_piece.py、夹爪工具 | 改为固定供料取棋→抬升→转移→下降→松爪→撤退；明确阶段、超时和失败返回，执行失败立即中止 |
| contracts | 完全自行设计 | 上游格式互不兼容 | 建立跨模块统一数据对象，不继承任一上游的可变 Board |
| orchestration | 完全自行设计 | Koch play.py 的流程与 SO101 toolkit 的锁仅作参考 | 单一棋局提交者、机器人独占权、回合状态机、观测验证与故障恢复 |
| recording | 完全自行设计 | 两项目可参考调试输出 | 每次运行保存配置快照、标定版本、帧图、棋局版本、决策和执行结果，支持离线回放 |
| interfaces | 自写接口，后期借鉴语言工具 | SO101: chess_robot_ui_llm_v2.py、llm-tools/ | 首期命令行/服务接口，后接 JARVIS；语言层只提交受校验任务，不直接发关节命令 |
| configs、scripts、tests | 项目专用设计，复用工具思路 | SO101 scripts/ 与 Koch 视觉测试 | 参数集中管理；先支持离线、模拟和单次诊断，再引入实体回合 |
| assets | 按需参考，实物重新核验 | Koch: low_cost_robot/assets/ 的棋盘、棋子 STL | 作为结构参考；Koch 臂模型不充当 SO101 模型，棋盘尺寸与夹取结构以本机实测为准 |

## 三、不能直接搬过来的内容

- Koch 的 robot.py、dynamixel.py、MuJoCo 机械臂模型：机械结构与电机协议不同。
- Koch 的 game.py：硬编码 hardware_restricted，还进入神经网络第五输入通道；不可把这些位置复制为我们的禁手。物理不可达属于执行能力，必须与棋规合法性分开。
- Koch 的 location_to_move 只检查展平后索引范围，不能替代 row/col 分别越界检查；do_move 也不能直接当作完整合法性验证入口。
- Koch 的 update_board_circle：偏向新增黑棋检测，直接修改棋盘；且在检查 circles is None 之前已进行转换与遍历。需要重新组织识别输出及无检测结果处理。
- Koch 的 play.py：导入的 Vanilla* 类与实际构造名称不一致，不能当现成可运行主程序。
- SO101 的 board_model：使用 (index+0.5) 的棋格中心，五子棋应采用交点；9×9 最外侧交点跨度为 8 个间距。标定脚本与模型的原点、角点和跨度必须统一。
- SO101 的 tool_move_piece：记录 motors_stopped=False 等结果后，外层仍返回 ok=True；抓取 stall 也不能单独证明抓到了棋子。改为明确失败传播与后续验证。
- SO101 的 kinematics.py：求解器输出需要由适配层检查残差、限位与可执行性；不能把求解器返回角度视为落子已成功。
- 上游的标定文件、夹爪百分比、速度、高度、工作区、串口路径、URDF 与模型权重不能视为本机可用配置。

## 四、建议新增的业务文件（本次仅设计，不创建空代码）

```text
src/so101_gomoku/
  contracts/       models.py                  # 观测、棋步、任务、结果
  vision/          camera.py board_locator.py rectifier.py classifier.py stabilizer.py
  calibration/     board_geometry.py transforms.py calibration_store.py
  game/            board.py rules.py reconciliation.py
  ai/              candidates.py evaluation.py alpha_beta.py
  robot/drivers/   base.py lerobot_driver.py mock_driver.py
  robot/kinematics/ adapter.py
  robot/planning/  pick_place_plan.py workspace.py
  robot/primitives/pick_and_place.py
  orchestration/   controller.py state_machine.py recovery.py
  recording/       session_recorder.py replay.py
  interfaces/      cli.py service.py
```

无需现在引入 ROS、消息总线、独立微服务或语言模型依赖。首期同一进程调用模块，结构保持可替换。

## 五、必须先固定的接口

- BoardObservation：9×9 单元状态、逐格置信度、frame_id、capture_time、calibration_id。unknown 只属于观测，不进入规则棋盘。
- GameState：只含已确认的 empty/black/white、next_player、version、终局结果；黑白身份独立于人类/机器人角色。
- Move：row、col、color；行列 0..8。建议 row 从校正图像上向下、col 从左向右，物理坐标通过标定显式转换，禁止隐式翻转。
- BoardGeometry：行列数、间距、原点与轴定义；长度内部统一米，角度在驱动/IK边界显式转换。
- RobotTask：task_id、expected_game_version、供料位、目标棋步、calibration_id。
- ExecutionResult：任务状态、失败阶段、原因、实际反馈。动作完成与视觉确认分开；成功发出命令不等于棋子已落稳。
- choose_move(state, deadline) → Decision；plan(task) → MotionPlan；execute(plan) → ExecutionResult；observe() → BoardObservation。

## 六、回合闭环与提交规则

启动检查→观测棋盘→等待人类→稳定观测→验证恰有一颗合法新增棋子→提交人类棋步→终局检查→AI 决策→目标可执行检查→记录待执行任务→机器人取放→撤至观察位→重新稳定观测→验证与预期全盘一致→提交机器人棋步→终局检查→下一回合。

- 人类观测出现多颗新增、移走或变色，进入异常确认，不擅自修正棋局。
- 机器人动作前保持旧棋局与待执行棋步；只有视觉确认后才提交新版本。
- 动作超时、目标未出现、落错位置、其他棋子移动或观测含 unknown 时暂停，记录证据。
- 不自动重复取放：先确认物理棋盘与夹爪状态，防止重复放棋。恢复必须重新对账。
- 机器人锁覆盖整次取放；UI、诊断与语言工具不能绕过同一控制入口。
- 若某交点物理不可达，先修正工作区/摆位或明确演示范围，不偷偷改成五子棋禁手。AI 可以在合法且可执行集合中选点，但不能据此声称覆盖全盘。

## 七、最快实施顺序与通过条件

1. contracts + game + mock_driver：纯软件能交替落子并正确判胜；覆盖越界、重复落子、长连和满盘。
2. calibration + IK adapter + 单次取放：用本机标定验证九个代表点，完成固定供料到指定交点；记录落点误差。
3. vision：使用保存的空盘、黑白棋与遮挡图片输出全盘状态；确认 unknown 不会被当成空位。
4. ai：先具备立即成五和阻止对手成五能力，再加入限时 Alpha-Beta；输出始终为合法候选。
5. orchestration + recording：先模拟并回放正常、超时和验证失败，再完成实体闭环；失败后不继续下一回合。
6. interfaces：最后连接 JARVIS 与自然语言工具。

## 八、依赖与复用边界

底层 LeRobot 应作为固定版本依赖，通过本项目适配层接入；不要默认安装整个参考仓库的训练、UI 与 LLM 依赖。现有本机 IK 如已验证，优先复用，避免首期并行维护两套求解器。引用或改编文件时保留上游版权和许可证。此文的分类是实现决策，不代表上游已通过运行验证。
