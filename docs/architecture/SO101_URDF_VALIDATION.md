# SO101 官方 URDF 离线验证

2026-11-16。模型来源：[TheRobotStudio 官方 SO-ARM100 仓库的 SO101 目录](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101)。文件保存在 assets/robot/so101/，完整 URDF、13 个引用网格、官方 README 及 Apache-2.0 许可证已下载。本次无需第三方镜像，官方原始文件通道下载成功。main 快照，未固定提交。

## 结论

IKPy 4.5.0 已成功在 Windows/Python 3.12 下加载官方 follower 新标定模型。可作为后续 IK 后端；当前尚未实现运行时 IKPy 适配器或发送真机命令。

验证通过：
- URDF 父子引用、单一根 base_link、无环与五关节末端路径。
- 五个机械臂转动关节的限位完整；夹爪开合未进入机械臂 IK 链。
- 末端为 gripper_frame_link，包含官方固定工具坐标变换，而非直接使用 gripper_link。
- 13 个网格引用全部存在。
- 独立解析 URDF 原点/旋转轴的正解与 IKPy 正解一致。
- 20 个模型可达姿态，用扰动初值做完整姿态 IK，回代全部通过；本次最大位置残差约 8.22e-8 米、旋转残差约 1.58e-9 弧度。这是数值一致性，不是实体精度。
- 远端不可达测试残差约 16.806 米，说明返回角度不能作为成功依据。

## 模型关节限位（非实测电机限位）

| 关节 | 近似范围（度） |
|---|---|
| shoulder_pan | -110 ～ 110 |
| shoulder_lift | -100 ～ 100 |
| elbow_flex | -96.83 ～ 96.83 |
| wrist_flex | -95 ～ 95 |
| wrist_roll | -157.21 ～ 162.79 |

## 模型注意事项

官方 README 区分新旧标定的零位；本次选择 so101_new_calib.urdf。夹爪 URDF 使用 revolute 角度，而 LeRobot 夹爪使用 0–100，官方说明二者映射尚未反映在模型中。固定末端关节有多余 axis，IKPy 警告后忽略；未擅改官方原件。官方还说明部分基座碰撞网格的处理，这次验证不构成碰撞模型验收。

## 还需验证

1. 本机关节读数与 URDF 零位/方向一致，以及机械改装、夹爪抓取点偏移。
2. IKPy 弧度向项目角度接口映射，明确固定与活动关节，并保留限位和位置/姿态残差检查。
3. 独立测量末端位置，验证真实 FK/定位误差；当前测试目标由同一模型生成，初值靠近目标，不能证明全局收敛或硬件准确。
4. 棋盘到基座标定后，验证九个代表点、供料点和接近方向；不得把模型可达误认为实机可安全放置。
5. 连续路径、速度、碰撞和密集棋子间隙仍需单独验证。

重跑命令：

```powershell
.venv/Scripts/python.exe -X utf8 scripts/diagnostics/validate_so101_urdf.py
```

原始数值与每组姿态见 data/so101_urdf_validation.json。
