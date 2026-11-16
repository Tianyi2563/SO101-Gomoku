# Assumption Ledger
ASK: never; assurance: draft

| ID | 问题 | 决策 | Class | Source |
|---|---|---|---|---|
| A01 | 编码与坐标 | 9×9 默认、0 空/1 黑/-1 白；unknown 仅观测；行向下列向右，内部米与关节度 | interface | default |
| A02 | 复用方式 | 少量纯函数改编放业务目录；依赖密集的 SO101 源码原样放 vendor 子目录作参考，不动态导入；运行驱动使用本机已安装 LeRobot | interface | default |
| A03 | 本机几何参数未知 | 不提供默认真实落子几何、工作区或夹爪值；demo 使用明确标记模拟的几何与标定 | semantic | default |
| A04 | 本次范围 | 优先移植/适配，完整 AI、ArUco、回合控制、视觉模型与 JARVIS 待后续；非交付完整下棋系统 | semantic | default |
| A05 | 执行成功含义 | waypoint 超时/目标被裁剪/抓取验证失败即停止；动作完成不等于视觉确认；失败不自动重试 | semantic | default |
| A06 | 依赖管理 | 沿用 .venv；numpy/OpenCV 为离线依赖，LeRobot/placo 按需导入，不自动安装训练/语言/UI 依赖 | interface | default |
| A07 | 驱动到位与抓取 | 使用关节目标误差而非静止判到位；抓取验证要求外部回调，mock 明确模拟；不承诺碰撞规划或硬件急停 | semantic | default |
| A08 | 标定定义 | 从棋盘平面 XY 到基座 XYZ 的刚体变换，至少三个非共线对应点拟合，不代表相机外参 | interface | default |

## Executor sweep 补充
| ID | 问题 | 决策 | Class | Source |
|---|---|---|---|---|
| A09 | 观测时效与版本绑定 | 内容核对函数只检查棋盘；时效、棋局/标定版本由未来 controller 校验，当前不宣称闭环具备该能力 | semantic | sweep |
| A10 | 阻塞与异常中止 | 抓取回调自身遵守 timeout；中止仅停止后续命令，不代表硬件急停/自动撤退；数据按 tuple 接口输入 | interface | sweep |

## Fresh reviewer sweep 补充
| ID | 问题 | 决策 | Class | Source |
|---|---|---|---|---|
| A11 | 规则变体 | 首期自由五子棋：黑白至少连续五颗获胜，长连有效，无禁手（沿用设计方案） | semantic | sweep |
| A12 | 多执行器控制权 | 默认锁只保护单个执行器；多个执行器或 UI 共用机械臂必须显式传入同一 control_lock，完整全局调度待 controller | interface | sweep |

## IKPy 接入阶段（2026-11-16，执行前约定）
| ID | 问题 | 决策 | Class | Source |
|---|---|---|---|---|
| IK01 | 求解器与坐标 | IKPy 4.5.0，显式 base_link→gripper_frame_link，按名称映射五个机械臂关节；外部度/米，内部弧度，不含夹爪开合 | interface | default |
| IK02 | 朝向语义 | 默认完整姿态 all；可显式选 X/Y/Z 约束工具单轴，IKAdapter 使用相同模式核验夹角，不以单轴成功冒充完整姿态成功 | semantic | default |
| IK03 | 模型与实物 | 离线采用 URDF 限位，实测限位可进一步收紧；零位/符号/工具点未实测，不自动连接硬件，不以离线结果宣称可下棋 | semantic | default |
| IK04 | 求解可靠性 | 当前关节为初值，有限优化预算，结果由 FK 残差与限位共同判定；不保证全局收敛，不隐式重试或改变朝向模式 | interface | default |
