# 五子棋 AI 与完整回合调度

更新：2026-11-17。软件功能已实现并离线测试，无相机/机械臂连接，无实际棋力、识别准确率或硬件可靠性结论。

## 可以直接玩

在仓库根目录的 PowerShell 运行：

```powershell
# 和参考项目的神经网络+MCTS下棋
.venv/Scripts/python.exe -X utf8 scripts/demos/play_gomoku.py --engine koch-mcts --size 9 --interactive

# 和本项目Alpha-Beta下棋
.venv/Scripts/python.exe -X utf8 scripts/demos/play_gomoku.py --engine alphabeta --interactive

# 自动模拟5个人机回合
.venv/Scripts/python.exe -X utf8 scripts/demos/play_gomoku.py --engine alphabeta --rounds 5
```

输入 `行 列`（0..8），如 `4 4`；`q` 退出。X黑棋，O白棋；当前默认机器人黑先行、人类白后手（2026-11-17更新）；`--human black` 可显式切回人类黑先行。默认每步0.2秒软搜索预算，可用 `--time-limit 1` 调整。MCTS默认64次playout，可用 `--playouts 200` 调整。预算不是可中断的硬实时承诺。

## AI实现与来源

| 引擎 | 实现 | 边界 |
|---|---|---|
| alphabeta（默认） | ai/candidates.py、evaluation.py、alpha_beta.py；邻域2候选、攻防评分、迭代加深negamax/Alpha-Beta；默认深度3、每层最多12候选；先查全盘立即获胜/必须防守 | 无需模型；超时返回上个已完成深度或合法启发式候选。双威胁可能无法全部挡住，不宣称最优或高棋力 |
| koch-mcts（显式选择） | 实际导入vendor/koch的VanillaPolicyValueNet与TreeNode，加载assets/models/koch/Vanilla_MCTS.model；项目适配层负责遍历、合法棋步和预算 | 仅9×9，torch CPU；复用预训练权重，不重新训练；原模型第五通道的硬件限制改为0，会改变训练输入分布，棋力未评估 |

上游源码来自 references/Koch_Gomoku_robot/Gomoku_AI/Vanilla_MCTS，MIT版权与许可证保留。vendor 内原文件不修改；运行不调用上游 VanillaBoard、get_action 或 play.py。模型通过 `weights_only=True` 和 CPU map_location 加载，保留五通道编码及行翻转；last_move通道依靠GameState新增字段，未知历史时为0。删去旧机械臂禁区、禁手和训练随机噪声；访问次数优先确定性选点，根节点每步重建；终局在网络推理前判断。若依赖/模型出错会明确失败，不静默换引擎。

公共接口：`engine.choose_move(state, time_limit_s)` → Decision，含move、engine、reason、score、completed_depth、nodes、elapsed_s。两种引擎score含义不同（启发式分数/MCTS估值），不得横向视为同一尺度。CPU线程数仅在demo/测试显式设1，库不更改全局线程配置。

## 回合状态机

`boot → wait_human → thinking → executing → verifying → wait_human/finished`

任何步骤失败进入 `paused`。GameController只读state属性是已确认棋局；RobotTask含task_id、move、expected_game_version、calibration_id。

1. start()：要求全盘与当前棋局一致，默认2帧稳定、置信度≥0.9，同标定，时间递增；启动棋盘异常就暂停。
2. run_turn()：读取人类新棋，无新增则继续等待；只允许恰好一颗合法新增，确认后提交人类棋步。人类赢棋直接终止，不调用机器人。
3. AI选点：检查合法性，并在AI计算后再次确认全盘未变化；生成待执行任务但不提交机器人棋步。
4. execute(task)：任务带版本，必须返回相同task_id且completed；失败保留待定任务并暂停，不自动重试。
5. 机器人动作结束后读取全新、稳定全盘观测，验证与预期棋局完全一致后才提交机器人棋步；返回visual_confirmed=True也不能替代controller重新观测。
6. recover(robot_stopped=True)：仅显式确认机器人已停后恢复；新棋盘必须匹配旧棋局或待定棋步后的棋局。后者可提交已完成落子，前者回到等待/思考，但recover本身不执行任何任务。其他变化保持暂停。

机器人第一手、人类/机器人终局、执行超时/失败、错任务ID、错盘、标定变化、旧帧、unknown、低置信度、AI报错和控制器并发均有对应处理。默认最多读取8次，unknown/低置信度允许有限重采；高置信非法变化直接暂停。每个观测阶段只接受阶段开始后采集的新帧，拒绝AI思考期间或恢复确认前缓存的帧；观测最长5秒，允许未来时间偏差0.5秒；真实相机应使用同一时钟域的采集时间，并保证读取的是新帧。等待帧的阻塞超时由相机接口实现。

## 接入真实模块的位置

- camera.observe() → BoardObservation：需接上后续棋盘识别；现有camera.py只采集图像，不能代替识别器。
- robot.execute(RobotTask) → ExecutionResult：可用 orchestration/executor.py 的 TaskExecutor。它调用 planner(task) → Waypoint序列，再调用现有PickAndPlace.execute(task_id, waypoints)。planner利用实测棋盘到基座变换与固定供料配置，不能用demo几何替代。
- 所有外部控制必须经过同一controller；共享机械臂的多个执行器还需共用底层控制锁。这里没有跨进程全局锁，也不替代急停。
- 回调需自行实现有限执行/采集时间，controller不能杀死一个阻塞中的硬件调用。
- 事件和待执行任务当前保存在内存，demo导出JSON；进程重启后的持久化对账恢复尚未实现。

## 验证与输出

```powershell
.venv/Scripts/python.exe -X utf8 -m unittest discover -s tests -v
.venv/Scripts/python.exe -X utf8 scripts/demos/play_gomoku.py --engine koch-mcts --size 9 --rounds 2
.venv/Scripts/python.exe -X utf8 scripts/demos/play_gomoku.py --rounds 5 --fail-on-round 2 --output data/sessions/gomoku_failure_simulation.json
```

当前累计47项测试通过。Alpha-Beta五回合：10个已确认棋步/5个机器人任务；MCTS两回合：4个已确认棋步/2个机器人任务。额外验证了终端交互、机器人先行和第二回合注入失败；失败例只提交了此前有效棋步，不会假提交失败机器人棋步。

产物在data/sessions/：gomoku_alphabeta_simulation.json、gomoku_koch_mcts_simulation.json、gomoku_failure_simulation.json、gomoku_interactive_simulation.json、gomoku_robot_first_simulation.json，明确标注OFFLINE_SIMULATION。观测、抓取和物理棋盘为合成数据；AI本身是实际搜索/推理。初始F0 smoke文件仍标明占位，不作为结果。

## 还未完成

真实相机棋子分类/遮挡判断、实物标定和供料参数、连续安全轨迹/碰撞、完整实物回合验收、网页控制与重启恢复。此阶段的软件闭环不能代替这些工作。


## 最终验证补充
累计47项测试通过；软件自动对弈已完整运行至终局（此运行27个已确认棋步，13个机器人任务，非棋力测评）。有界复核发现并修复了AI思考期间缓存帧可被执行前核对接受的问题，新增两项缓存帧回归；恢复阶段同样要求确认后新采集帧。复核为同模型家族provisional，不是硬件验收。
