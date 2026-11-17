# SO101 五子棋

单臂机器人执黑先手、人类执白。棋盘为 **9×9方格、10×10交点**，落子坐标0..9。格距暂按用户提供的约21mm配置，交点跨度约189mm，打印后尺寸仍需实测。

## 当前进度（2026-11-17）

- 已完成规则、Alpha-Beta AI、完整回合状态机、动作后验证提交、异常暂停及显式恢复。
- 已接入官方SO101 URDF与IKPy，提供模型内限位/位置/朝向检查及连续轨迹生成。
- 已有交点黑/白/空/unknown识别首版，采用空盘参考和手选四个外侧交点，目前仅合成图测试。
- 已实现手动标定录入、拟合、保存/加载和独立验证误差报告。
- 已实现事件/观测/可选原图持久化、只读回放、进程退出后的棋局对账，恢复不会自动重发机器人任务。
- 最近 **76项离线测试通过**。未连接真实相机或机械臂；没有实拍准确率、真机定位精度或抓取成功率结论。

## 快速运行

使用已有Python环境或安装所需依赖后，从仓库根目录运行。测试环境使用Python3.12、numpy、SciPy、OpenCV、IKPy4.5.0；可选Koch引擎需要PyTorch。依赖安装与完整真机环境仍需按目标机器核对。

```powershell
# 10×10交点的软件人机对弈，机器人黑先
.venv/Scripts/python.exe -X utf8 scripts/demos/play_gomoku.py --engine alphabeta --interactive

# 回归测试（不打开硬件）
.venv/Scripts/python.exe -X utf8 -m unittest discover -s tests -v

# 标定、模型轨迹、日志与重启恢复的合成示例，自动生成新目录
.venv/Scripts/python.exe -X utf8 scripts/demos/offline_tools_demo.py
```

输入行列如`4 4`，`q`退出。Koch预训练NN+MCTS也已接入，但模型只支持9×9交点，仅供历史参考：

```powershell
.venv/Scripts/python.exe -X utf8 scripts/demos/play_gomoku.py --engine koch-mcts --size 9 --interactive
```

不能将该9×9模型直接用于当前10×10实体棋盘。

## 文档与示例

- [当前实现状态](docs/architecture/IMPLEMENTATION_STATUS.md)
- [对照初始规划的推进清单](docs/architecture/PROJECT_PROGRESS.md)
- [AI与回合调度](docs/architecture/AI_AND_TURNS.md)
- [交点视觉与照片要求](docs/architecture/INTERSECTION_VISION.md)
- [离线标定、轨迹、日志与恢复工具](docs/architecture/OFFLINE_TOOLS.md)
- [IKPy接入](docs/architecture/IKPY_INTEGRATION.md)
- [合成回放页面](data/offline-tools/final-20261117/replay.html)（下载后用浏览器打开）
- [待填写标定点模板](data/calibration/points-21mm-template.json)
- [第三方代码与许可证](THIRD_PARTY_NOTICES.md)

## 下一步

实测棋盘与棋子尺寸、采集同机位真实图片、核对本机关节零位/方向/TCP并录入示教点，然后验证单次实体取放与连续回合。模型轨迹当前未做完整碰撞检查，也未自动作为定时真机执行流。
