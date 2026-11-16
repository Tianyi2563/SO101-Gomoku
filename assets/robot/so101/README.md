# 官方 SO101 模型

来源：https://github.com/TheRobotStudio/SO-ARM100/tree/main/Simulation/SO101
下载：2026-11-16，main 分支快照（未固定 commit）。官方文件保持原样，Apache-2.0 许可证见 LICENSE；每个附属资源的来源和大小见 DOWNLOADS.json。

URDF：https://raw.githubusercontent.com/TheRobotStudio/SO-ARM100/main/Simulation/SO101/so101_new_calib.urdf

采用 follower 新标定模型，非 leader。五个机械臂关节：shoulder_pan、shoulder_lift、elbow_flex、wrist_flex、wrist_roll。IK 链从 base_link 到 gripper_frame_link，末端通过固定 gripper_frame_joint 接到 gripper_link；gripper 开合分支不加入 IK。

注意：官方固定关节携带 axis 字段，IKPy 会警告并忽略；不影响已验证的固定变换，原文件未改动。官方 README 说明新旧标定零位不同，URDF 夹爪角度与 LeRobot 0–100 开合值没有直接统一。新标定不等于本机已经匹配：零位、方向和抓取点需实物核验。

验证：scripts/diagnostics/validate_so101_urdf.py；结果 data/so101_urdf_validation.json。仅检查模型结构、资源、独立 FK 和模型内 IK 回代，无碰撞、真实精度或棋盘可达性验收。
