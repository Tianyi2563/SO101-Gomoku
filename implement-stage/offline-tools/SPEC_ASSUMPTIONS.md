# Offline tools specification
2026-11-17. ASK never; balanced/draft. Base commit e742a04ba58495ab8fe230e94a3d23fe656839b5; existing working tree changes preserved.
Target: 标定点录入/拟合/持久化/误差报告、模型内连续轨迹/检查、完整事件和观测日志/回放/重启对账及扩展回归。无设备连接。
PDF read-only inspected with Poppler render; 9x9 squares/10x10 intersections. Source retained assets/board/user_board.pdf.

| ID | Decision | Class | Source |
| O01 | 21mm是用户近似输入，10交点跨度189mm；配置标记未实测，PDF布局不充当打印后精度或相机照片 | semantic | user |
| O02 | 标定点以row/col+基座XYZ输入，CLI毫米→内部米；拟合与独立验证点分组且坐标不交叉，至少3非共线拟合点+1验证点；报告区分拟合残差与验证误差 | interface | default |
| O03 | 标定文件schema1含ID/来源/几何/变换/误差/阈值，原子写入加载校验；合成示例明确synthetic，FK取点不是外部精度证据 | semantic | default |
| O04 | 轨迹位置线性、旋转Slerp、时间五次平滑；逐样本IK/FK/限位/工作区/关节跳变检查，整体拉长时间满足离散速度和加速度限制；有限采样不证明扫掠碰撞安全，不发送硬件 | semantic | default |
| O05 | 会话JSONL顺序记录状态、待定任务、事件、完整观测；可选保存PNG原始图像；意图记录flush+fsync后才执行，异常不自动重发；单写者OS文件锁，日志错误fail-closed | interface | default |
| O06 | 重启从验证过的日志恢复到PAUSED；必须人工确认robot_stopped并以新观测对账；校准ID与棋盘参数不得静默替换，已落待定棋可提交，旧盘可清除待定任务，不自动执行 | semantic | default |
| O07 | 损坏完整记录或序号/状态转换不合法拒绝回放；默认拒绝截断尾记录，显式repair-tail可备份后去掉未完成最后一行，不修复其他损坏 | interface | default |
| O08 | 无真实基座点、布置/供料/关节限位/速度数据，示例全为model/synthetic；不提供可直接执行的真机标定或轨迹；全局锁/急停/完整碰撞检测不在本轮 | semantic | default |
| O09 | 图像仅在BoardObserver配置image_sink且capture发生时保存；此前未记录原图的会话不能凭空回放图像；离线回放不导入或调用硬件 | interface | default |

# Ladder
F0 offline smoke entry+JSON; F1 calibration workflow+tests; F2 trajectory+tests; F3 durable session/replay/restart+tests; F4 demos, expanded regression and review.
Success: unittest discover; offline_tools_demo.py; calibration and replay CLIs.
| O10 | 若进程在“人类已落白棋、提交记录尚未落盘”时退出，无pending机器人任务且轮到人类时，recover可显式accept_human_move=True接纳恰一颗合法人类新增；默认仍拒绝，不自动执行机器人，其他变化保持暂停 | semantic | default |

# Build record
F0 smoke exit0. F1 calibration3 tests exit0. F2 trajectory3 tests exit0, including officialSO101 short path. F3 session+controller and OS-process-exit tests pass. F4 final full suite76 tests exit0, compileall exit0.
Actual CLI init/fit/check, trajectory generation and read-only replay/export run. Final examples data/offline-tools/final-20261117, report summary.json. 16 trajectory samples; 40 journal records, 14 synthetic images; restart before/after task count2, reconciled stateversion3. HTML initial and selected observation screenshots visually inspected using headlessEdge; no real device access.
Review round1 found3 lifecycle/config/phase persistence issues; fixed with regressions. Focused round2 verified3 fixes and28 tests. Same-family provisional, not independent acceptance. Later explicit-human recovery and viewer escape tests verified by executor; no third review claimed. One initial viewer escaping regression failed during development, corrected and rerun before final76 green.
Scope limits remain: near21mm not measured, synthetic calibration not hardware accuracy, sampled trajectories not collision proof or timed hardware execution, journal does not guarantee all power-loss/storage-corruption recovery.
Worktree changes retained, no automatic commit/push. No unresolved reported review findings.
