# IKPy SO101 接入与测试

2026-11-16。已实现 src/so101_gomoku/robot/kinematics/ikpy_kinematics.py，并接入现有 IKAdapter 和 PickAndPlace 的求解接口。采用 IKPy 4.5.0，无 Placo 运行依赖，不连接硬件。

## 接口约定

- 外部关节角为度、位置为米；内部转换为弧度。恰好五个机械臂关节，拒绝夹爪百分比混入角度向量。
- 通过关节名映射 IKPy 链下标；固定基座与工具关节不参与优化。
- 官方模型从 base_link 到 gripper_frame_link；本机抓取点偏移和校准方向仍需核实。
- 默认 orientation_mode='all'，检查完整朝向；显式 X/Y/Z 模式仅约束同名工具轴方向。绕该轴的旋转是自由的，不能把它称为完整姿态一致。
- 求解器和校验器必须使用同一关节顺序、同一朝向模式。不会失败后偷偷放宽约束。
- 当前角度作为初值，设置优化预算；预算不是严格墙钟超时，当前不保证全局收敛或控制周期。
- raw inverse_kinematics 返回候选解；必须通过 IKAdapter.solve 检查结果限位与位置/朝向残差后才能用于后续动作。

## 最小离线调用

从仓库根目录使用可导入 src 的 Python 环境：

```python
from pathlib import Path
from so101_gomoku.robot.kinematics.ikpy_kinematics import IKPyKinematics, SO101_JOINTS
from so101_gomoku.robot.kinematics.adapter import IKAdapter

solver = IKPyKinematics(Path('assets/robot/so101/so101_new_calib.urdf'))
ik = IKAdapter(solver, SO101_JOINTS, solver.limits_deg,
               position_tolerance_m=1e-4, orientation_tolerance_rad=1e-3)
# 仅为离线模型生成目标，不是实际待发送角度。
current_deg = [0., 0., 0., 0., 0.]
target_pose = solver.forward_kinematics([1., -1., 1., 1., -1.])
result_by_joint_name = ik.solve(current_deg, target_pose)
```

上例使用 URDF 限位和数值测试容差，不是已标定的硬件配置。若实测限位更窄，向 IKAdapter 提供实测范围；本机当前关节、运动方向和工具点必须先对齐模型。

## 验证结果

- 全仓库单元/集成测试：22 项通过（原15项 + 新7项）。
- 新7项包含：度/弧度回代、按名称重排、输入和限位、不可达拒绝、轴约束及校验模式匹配、绕轴自由度校验、真实 SO101 模型 IK→现有执行器→模拟电机。
- 额外数值检查：完整姿态20例 + Z轴方向20例，共40例通过项目适配器；两种模式均拒绝远端不可达目标。
- 最大位置残差约3.10e-8米，最大受约束朝向残差约2.98e-8弧度。这些是同模型生成目标、近初值的数值一致性结果，不是实物精度、全局收敛率或棋盘覆盖率。
- 执行器对不可达目标在 preflight 阶段中止，无后续模拟电机命令。现有默认 reuse_demo 仍保留原合成 IK，不暗中改变其演示含义。
- 官方 fixed 关节带 axis 的警告仍会显示：IKPy 忽略该字段，原模型未修改。

```powershell
.venv/Scripts/python.exe -X utf8 -m unittest discover -s tests -v
.venv/Scripts/python.exe -X utf8 scripts/diagnostics/check_ikpy_integration.py
```

数值结果：data/ikpy_integration_report.json。

## 下一步

先核对本机电机零位/符号、实际工具点与关节限位，再采集棋盘到基座变换。随后用九个代表交点和固定供料点检查位置与接近方向，生成连续轨迹并验证碰撞和速度。现阶段没有真实棋盘可达性结论，也没有真机控制验收。
