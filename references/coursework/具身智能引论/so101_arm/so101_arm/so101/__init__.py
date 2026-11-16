"""SO-ARM101 机械臂课程验收工具包。

模块划分：
    urdf_model  : 解析 so101_new_calib.urdf，得到关节链 / 限位 / 零位
    kinematics  : 正运动学 FK、雅可比、解析 IK（两连杆）、数值 IK（阻尼最小二乘）
    frames      : 关节角单位换算（lerobot 动作值 <-> URDF 弧度）
    backend     : 硬件后端（真机 lerobot / 模拟 Mock）
    trajectory  : 姿态之间的时间插值（线性 / 最小 jerk / 梯形速度）
    arm         : 高层控制器：目标姿态 -> 插值 -> 限速 -> 下发
    presets     : 预设姿态与预设动作的存取、录制与播放
    keys        : 键盘输入（Windows msvcrt / 可选 pynput）
    voice       : 语音识别（faster-whisper）+ 指令解析
    vision      : 摄像头黑球检测 + 视觉闭环控制
"""

__version__ = "0.1.0"
