# Assumptions
ASK: never; effort: balanced; assurance: draft
| ID | 问题 | 选择 | Class | Source |
|---|---|---|---|---|
| T01 | 引擎与规则 | 自由五子棋 >=5 连赢、无禁手；默认限时 Alpha-Beta，显式可选 Koch NN+MCTS；不静默切换引擎 | semantic | default |
| T02 | 模型限制 | Koch 模型仅9×9、CPU推理，保留五通道/行翻转编码；旧硬件限制通道改0，不用旧禁手；未评估预训练模型棋力 | semantic | default |
| T03 | 搜索预算 | Alpha-Beta迭代加深、邻域2候选、默认最多12候选/层、深度3、0.5秒搜索软预算；全盘必胜/必防先检查，超时返回已完成深度或合法启发式候选；不是硬实时保证 | semantic | default |
| T04 | MCTS修改 | 复用上游树/网络架构与权重，修复终局先验顺序、删硬件禁手、确定性访问最多选点、每次新树；额外必胜/必防；预算按playout数与软时间控制 | interface | default |
| T05 | 角色与观测 | 默认人类黑/机器人白；支持显式互换。注入camera.observe()产生稳定候选观测，controller仍要求连续2帧、高置信、同标定、新时间戳，默认最多8次读取；单帧异常暂停，unknown/低置信仅在有限重采后暂停 | semantic | default |
| T06 | 棋局提交 | 单一controller拥有只读GameState；初始化需全盘匹配；人类新增合法棋确认后提交；机器人待执行棋不先写入，只在动作完成+新全盘验证后提交；终局禁止再执行 | semantic | default |
| T07 | 任务与异常 | task含唯一ID、棋步、棋局/标定版本；robot.execute(task)→ExecutionResult，id匹配；失败/超时/错盘暂停，不自动重试；recover只接受棋盘等于旧棋局或已执行待定棋步，不能修改其他棋子 | interface | default |
| T08 | 同步与权限 | controller同步非重入锁覆盖整个回合，接口/诊断需共用同一控制入口；不提供跨进程锁/硬件急停；接口应自行约束阻塞时间，不声称硬超时能中断硬件 | interface | default |
| T09 | 可恢复数据 | 在内存保存事件/待定任务，demo导出JSON；进程重启持久化恢复另做；GameState增加可选last_move供模型通道，无法获知历史时该通道为0 | interface | default |
| T10 | 执行接入 | 提供TaskExecutor用plan(task)连接现有PickAndPlace；demo默认物理棋盘合成、机械臂模拟；真实视觉与机器人仍由后续实测接入 | interface | default |
| T11 | 恢复与时效 | recover必须显式robot_stopped=True，确认机械臂已停；稳定新观测只允许旧棋局或待定棋步后棋局。默认观测最长5秒、未来偏差0.5秒、动作后观测晚于执行结束，进程内时间戳严格递增，frame_id与紧邻上一帧不同（不要求字符串排序或历史全局去重）；无新增人类棋返回等待，不算失败 | semantic | default |
| T12 | 离线时钟 | Windows快速模拟可能产生相同wall-clock时间；SimulatedTable默认使用递增模拟时钟，demo与controller共用它。真实camera必须提供新帧采集时间，不放宽生产控制器的重复帧拒绝规则 | interface | default |
| T13 | 观测与执行间隔 | AI计算后、执行前再次确认全盘等于已提交棋局；若对手在思考时动棋或遮挡导致无法确认，暂停且不发出动作 | semantic | default |

| T14 | 阶段间缓存观测 | 新阶段读取观测时，以该阶段开始时间为采集下界；初始化/执行前/恢复不能用AI思考或人工确认前缓存的旧盘面作为证据，即使未超过5秒；旧帧暂停，camera需提供实时新帧 | semantic | sweep |
