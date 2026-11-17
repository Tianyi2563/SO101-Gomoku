# 不连接硬件的标定、轨迹与会话工具

2026-11-17。本轮未连接相机或机械臂。用户图样已只读渲染核对：9×9方格、10×10交点；原件保留assets/board/user_board.pdf。格距按用户约21mm暂定，外侧交点跨度189mm；打印缩放与X/Y实际间距尚未测量。当前几何假设等距正方网格，不能拿PDF页面比例当物理标定。

## 已完成

| 工具 | 代码 | 输出 |
|---|---|---|
| 标定点录入/拟合/保存加载 | calibration/workflow.py、scripts/calibration/board_calibration.py | 带ID/来源/单位/独立验证误差的JSON |
| 模型内连续轨迹 | robot/planning/trajectory.py、scripts/diagnostics/plan_trajectory.py | 时间、目标与FK姿态、关节角、夹爪端点动作、速度/加速度/残差报告 |
| 持久化会话 | recording/session.py、controller journal参数 | events.jsonl，配置、事件、全盘观测、棋局快照、待定任务 |
| 原始图像记录 | BoardObserver(image_sink=journal.record_image) | PNG原图及关联帧ID；仅实际传入的图像可记录 |
| 只读回放 | recording/replay_html.py、scripts/diagnostics/replay_session.py | 逐事件CLI/JSON时间线、自包含HTML播放器 |
| 重启对账 | GameController.resume_from_journal、recover | 重启固定暂停，新观测对账；不自动执行或重复发送任务 |

## 1 标定工具

现成待填写模板：data/calibration/points-21mm-template.json。所有base_xyz_m为null，故意没有伪造真机坐标。建议5个拟合点、4个独立验证点；至少3个非共线拟合点+1个独立验证点。拟合/验证不能重复同一交点，模板中未填点会阻止拟合。

创建新模板：

```powershell
.venv/Scripts/python.exe -X utf8 scripts/calibration/board_calibration.py init --file data/calibration/my-points.json --size 10 --spacing-mm 21
```

录入一条基座坐标（下方100/200/30只演示命令格式，不是实际点）：

```powershell
.venv/Scripts/python.exe -X utf8 scripts/calibration/board_calibration.py set-point --file data/calibration/my-points.json --row 0 --col 0 --xyz-mm 100 200 30 --role fit
```

全部填写后拟合并保存：

```powershell
.venv/Scripts/python.exe -X utf8 scripts/calibration/board_calibration.py fit --file data/calibration/my-points.json --output data/calibration/my-calibration.json --id board-measured-v1 --limit-mm 3
.venv/Scripts/python.exe -X utf8 scripts/calibration/board_calibration.py check --file data/calibration/my-calibration.json
```

CLI录入毫米，内部统一米。变换是board→robot_base，原点(0,0)交点，X沿列、Y沿行，板面Z=0。基座点应对应同一个实际TCP接触棋盘的位置。

source需如实标为manual_input/fk_model/synthetic；由FK计算的“测量值”和合成点不能作为外部精度证据。spacing_verified只在实际测过打印后格距时设true。模型不拟合缩放，格距设错可能使误差增大，应修正测量数据而不是放松阈值掩盖。

报告分别给fit与validation的RMS、最大值和逐点误差。3mm只是原规划初始验收目标，可调整；超限仍保存可检查的失败报告，但fit/check退出码为2。加载会重新拟合并核对变换及误差，损坏/不一致文件被拒绝。文件原子替换；保存前验证。通过数学阈值不等于获得已验证的真机标定。

## 2 模型内轨迹

离线示例目录：data/offline-tools/final-20261117/。MODEL_ONLY_trajectory_request.json提供明确标记模型内的请求（不是实际供料/棋盘布局）。

```powershell
.venv/Scripts/python.exe -X utf8 scripts/diagnostics/plan_trajectory.py --request data/offline-tools/final-20261117/MODEL_ONLY_trajectory_request.json --output data/trajectory-example.json
```

流程：TCP位置线性插值、旋转Slerp、五次平滑时间参数；逐样本以上一关节解为初值求IK；检查关节限位、残差、配置的工作区、关节跳变；以有限差分核对关节速度/加速度，并检查实际FK的线速度/角速度。必要时整体延长时间。夹爪动作只保留在对应路点终点。

报告含sample_count、持续时间、缩放比例、最大速率/加速度、FK残差、实际采样间隔。缩放后采样间隔可能大于请求的初始sample_period，不是已实现的固定周期真机控制流。

重要边界：

- 限位示例来自URDF，不是实测电机限位；示例速度和加速度也是离线测试值。
- 未指定workspace就明确报告workspace_checked=false；示例没有实际布局数据。
- collision_status始终明确NOT_CHECKED。有限采样、端点和TCP包围盒检查，不是网格自碰撞/棋子扫掠碰撞验证。
- 不验证电机动力学、载荷、跟踪误差或jerk，不会发送硬件命令。当前PickAndPlace未自动替换成定时轨迹执行器；真机执行/重采样仍需后续实现。

## 3 会话、图片与回放

`SessionJournal(session_dir, configuration=...)`提供单写者OS文件锁。header保存棋盘/角色/标定ID、额外配置、控制器阈值。每个事件保存序号、时间、完整棋局和待定任务；每次观测含全盘、逐点置信度、frame_id、采集时间和标定ID。意图记录flush+fsync成功后才调用robot.execute；磁盘写入错误阻止后续执行，严重写入失败可能向调用者抛错而非只返回phase。

原图需要显式接入：

```python
with SessionJournal('data/sessions/my-session', configuration=真实JSON配置) as journal:
    observer = BoardObserver(camera, detector, image_sink=journal.record_image)
    controller = GameController(observer, engine, executor, calibration_id, journal=journal)
    # 由应用显式调用start/run_turn；构造本身不连接任何硬件。
```

`BoardObserver`不会凭空提供原图；合成会话图片明确只是示意，未记录原图的历史事件不能回放真实画面。

只读回放：

```powershell
.venv/Scripts/python.exe -X utf8 scripts/diagnostics/replay_session.py data/offline-tools/final-20261117/SYNTHETIC_session --export data/replay-timeline.json --html data/replay.html
```

HTML可直接打开，支持播放/暂停、逐条事件、拖动时间线、查看已确认盘/当前观测盘、对应图像与记录。#37这样的URL片段可定位事件。回放不实例化驱动或执行器。

## 4 重启后的对账

```python
with SessionJournal(原会话目录, configuration=与原会话一致的配置) as journal:
    controller = GameController.resume_from_journal(
        journal, observer, engine, executor, 原标定ID)
    # 此时始终PAUSED，不执行机器人。
    controller.recover(robot_stopped=True)
```

- 新观测与已确认棋盘一致：保留棋局、清除待定任务；recover自身不执行，后续run_turn需应用显式调用。
- 与待定机器人棋步后的棋盘一致：提交这颗已放棋，不再次取放。
- 没有待定机器人任务、轮到人类、且人类在退出期间新增恰一颗合法棋：默认仍暂停；人工可显式 `recover(robot_stopped=True, accept_human_move=True)` 接纳。错误颜色、多颗/移动/缺失棋子或unknown仍被拒绝。
- 配置/标定ID冲突、完整日志损坏、非法状态/任务生命周期拒绝恢复。重新打开时提供不同configuration会报错；未提供则采用已保存配置，不静默换尺寸/格距。控制器恢复其保存阈值。
- 只丢了最后一条未完成行：默认拒绝；人工确认后可用 `replay_session.py 会话目录 --repair-tail`，备份尾部再去掉该未完成记录；不能修复其他损坏。
- 单会话单写者不等于同一机械臂跨进程全局锁，也不替代急停。进程退出恢复已有测试，不承诺任何断电/存储损坏场景都可恢复。

## 5 运行完整离线示例与回归

```powershell
.venv/Scripts/python.exe -X utf8 scripts/demos/offline_tools_demo.py
.venv/Scripts/python.exe -X utf8 -m unittest discover -s tests -v
```

demo默认创建新目录，避免覆盖既有日志。已生成的可查样例位于data/offline-tools/final-20261117，包含：

- SYNTHETIC_points.json / SYNTHETIC_calibration.json：5拟合+4验证的合成数学示例。
- MODEL_ONLY_trajectory_request.json / MODEL_ONLY_trajectory.json：官方SO101模型内短路径与速率检查。
- SYNTHETIC_session/events.jsonl及14张合成图、timeline.json、replay.html。
- summary.json：重启前后机器人任务数均为2，对账后棋局版本3；没有重复执行。

扩展回归覆盖标定单位/独立验证/错误文件，真实SO101模型短路径，速率/工作区/跳变，AI四方向双颜色战术、随机合法局面，白底白棋拒识，日志损坏/落盘失败/锁/截断，真实子进程os._exit后恢复，未提交人类棋的显式接纳，以及完整图像记录桥接。所有这些都是离线验证，不是实拍精度或真机成功率。

## 下一步仍需真实信息

实测格距（含打印X/Y方向）、基座示教点、真实TCP/关节零位/限位、供料位置和速度范围；相同机位空盘/黑白棋/遮挡照片。白底PDF与白棋可能缺乏可观测对比，需真实图像评估，当前不会宣称已解决。

最终回归：76项通过（data/offline-tools/final-20261117/tests.log）；图像回放页面已渲染检查。新增恢复边界和日志完整性有针对性回归，真实硬件/照片仍未验收。
