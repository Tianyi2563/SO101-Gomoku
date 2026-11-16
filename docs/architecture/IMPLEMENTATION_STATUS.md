# Implementation status

更新：2026-11-16；静态移植 + 离线验证；没有连接机械臂，也没有真实视觉识别验收。

## 已完成并离线检查

| 部分 | 文件 | 当前能力 |
|---|---|---|
| 统一数据 | contracts/models.py | Move、BoardObservation、ExecutionResult；unknown 不进入规则棋盘 |
| 五子棋规则 | game/board.py | 不可变状态、行列边界、占用、轮次、四方向连五/长连、满盘、终局拒绝落子 |
| 全盘核对 | game/reconciliation.py | 恰一颗新增合法棋、全盘一致验证；拒绝 unknown、低置信度和多格变化 |
| 棋盘几何 | calibration/board_geometry.py | 交点模型，去掉国际象棋半格偏移；尺寸参数显式传入 |
| 坐标标定 | calibration/transforms.py | 多点刚体拟合、拟合残差、独立验证点误差；非相机外参 |
| 视觉基础 | vision/geometry.py、stabilizer.py | 已知对应点透视校正、网格、多帧一致性；无自动棋子分类 |
| IK 适配 | robot/kinematics/adapter.py | 调用给定 FK/IK，检查关节限位、位置/姿态残差 |
| 动作规划 | robot/planning/pick_place_plan.py | 固定供料取放路径点、显式朝向/高度、工作区边界校验；没有碰撞/速度轨迹规划 |
| 动作执行 | robot/primitives/pick_and_place.py | 端点预检、共享控制锁、到位/松爪超时/抓取未确认中止；不自动重试、不更新棋局 |
| 模拟演示 | interfaces/demo.py | 脚本指定棋步→模拟取放→合成观测核对；明确不代表 AI、真实视觉或物理仿真 |

## 已放入对应目录，但未真机验证

- robot/drivers/vendor/lerobot_so101/：原样驱动参考副本；不独立运行。
- robot/drivers/lerobot_driver.py：调用本机 LeRobot 的适配器，显式连接，要求角度模式与已有标定；关节到位按误差判断，目标裁剪报错。
- robot/kinematics/placo_kinematics.py：原样 FK/IK 求解器，需 placo、本机 URDF 与适配器校验。
- vision/camera.py：OpenCV 相机采集封装，未打开相机。

本机环境检查：numpy/OpenCV 可用；LeRobot 包存在，但导入驱动链报缺少 torch；placo 不存在。未自动安装训练环境或替用户选择硬件参数。

## 还需要完成

1. 真机基础：修复 LeRobot 依赖，提供已有 IK/URDF、串口、关节标定、夹爪值、真实工作区与末端工具偏移。验证连接/关节回读。
2. 标定操作工具：采集示教点、保存/加载标定与版本；独立验证点测试、九点可达性/重复精度；已有拟合函数不是完整标定工作流。
3. 视觉核心：ArUco 自动定位、畸变校正、黑白/空/unknown 分类器、遮挡判断、真实图像阈值与置信度评估、固定俯拍连续观测。
4. 运动可靠性：将路径点接入受限速度的轨迹执行；碰撞与密集落子间隙、固定供料库存、真实抓取传感验证、释放与撤离观察姿态、急停和故障恢复。当前抓取回调需真实实现，不能使用 demo 的恒真回调。
5. 棋局 AI：候选点、棋形评估、限时 Alpha-Beta 与决策解释；未复制神经网络 MCTS 为首期默认 AI。
6. 系统闭环：带棋局版本和标定版本的任务、单一棋局提交者、人类回合与机器人回合状态机；验证后提交、失败对账后恢复。
7. 数据与界面：真实会话记录/回放、JARVIS 接口、自然语言工具；demo JSON 仅模拟示例。

## 运行

从仓库根目录使用 PowerShell：

```powershell
.venv/Scripts/python.exe -m unittest discover -s tests -v
.venv/Scripts/python.exe scripts/demos/reuse_demo.py
```

模拟产物：data/sessions/reuse_demo.json。各项离线测试只验证指定软件边界，不能用于宣称实际精度、识别率或抓取成功率。代码来源见根目录 THIRD_PARTY_NOTICES.md。

## 离线验证记录

本次最终基础回归共 15 项通过：规则/满盘/终局、观测多格变化与 unknown 拒绝、交点与刚体标定、透视几何、多帧稳定、IK 残差、驱动目标裁剪与不到位、抓放失败中止和共享锁。模拟 demo exit 0；业务目录 compileall exit 0。

复现环境：Python 3.12.10、numpy 2.5.3、opencv-python 5.0.0.93；已安装但驱动导入不完整的 lerobot 0.6.1。当前未修改 .venv 依赖。

## 补充边界

观测核对函数负责内容一致性，目前不自行校验 frame 的新鲜度、棋局版本与标定版本；这些需由待实现的 controller 管理。数据对象使用 tuple 作为接口约定，不能将任意外部列表不经转换直接作为已确认状态。夹爪抓取验证回调应自行实现超时，现阶段没有独立线程强制终止回调。异常中止表示停止后续命令，不等于机器人硬件急停或自动安全撤离。

多执行器约定：PickAndPlace 默认创建实例内锁；若多个执行器/界面共用机械臂，必须注入同一个 control_lock，不能将默认锁理解为进程级或全局独占。

## 最新 IK 模型验证（2026-11-16）
IKPy 4.5.0 已安装；官方 follower 新标定 URDF 与13个网格已放入 assets/robot/so101/。模型离线 FK/IK 验证20/20通过，详见 SO101_URDF_VALIDATION.md。尚需运行时适配、实物零位/工具点核对和棋盘标定。此前“无 URDF”状态已由此更新；没有进行真机验收。

## 最新 IKPy 接入（2026-11-16，替代此前“待适配”状态）
IKPyKinematics 后端已通过现有 IKAdapter 接入 PickAndPlace。完整姿态及显式单轴约束均可用，验证模式一致；外部度、内部弧度，五关节按名称映射。22项测试通过，额外40个模型内案例通过。真实模型IK+模拟电机测试已运行；默认 demo 仍为合成 IK。详见 IKPY_INTEGRATION.md。
仍缺：本机零位/方向与工具点核对、实测限位、棋盘标定与九点可达、完整轨迹和真机验收；未连接机械臂。
