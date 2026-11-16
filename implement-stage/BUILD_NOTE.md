# Build note
| Rung | 内容 | 验收 | Tier | 状态 |
|---|---|---|---|---|
| F0 | 模拟入口与结果文件 | python scripts/demos/reuse_demo.py | MUST | passed |
| F1 | 规则、棋盘几何和标定 | unittest tests.test_core | MUST | passed |
| F2 | 视觉几何与 IK/驱动归档适配 | unittest tests.test_core tests.test_adapters | MUST | passed (offline) |
| F3 | 路径、失败中止、mock 抓放演示 | unittest discover + demo | MUST | passed (offline) |

## Run record
现有 .venv Python 3.12.10。F0 demo exit 0；F1 四项测试 exit 0；F2 七项测试 exit 0；F3 十三项测试 exit 0 + demo exit 0，模拟输出 data/sessions/reuse_demo.json。
完整驱动导入探测失败：LeRobot 存在但缺少 torch；placo 缺失。硬件集成不在本次离线通过结论内。
无根 Git 仓库，未初始化；源码文件清单使用 rg --files src tests scripts/demos。

## Deferred
完整 AI、ArUco、真实黑白分类、完整回合控制、会话记录/回放、标定交互/持久化、网页及所有真机验收。详见 docs/architecture/IMPLEMENTATION_STATUS.md。

## Blockers
离线无阻塞。硬件阻塞：依赖不完整、无本机 IK/URDF 接入与真实标定/抓取回调。未连接硬件。

最终补充回归：15 项测试通过（exit 0）；一次新增测试代码段位置错误修复后重跑通过，不涉及业务逻辑失败。demo exit 0，compileall exit 0。当前环境版本见 IMPLEMENTATION_STATUS.md。

Fresh native reviewer: 有界静态复核未报明确代码缺陷，发现 2 项未声明约定（1 semantic：自由五子棋长连；1 interface：多执行器共享锁）。已补 ledger A11/A12 和状态文档。审阅同家族、provisional；修订后由执行者核对，未宣称独立验收或 clean re-review。不存在未记录的审阅发现。

## IKPy 接入阶段记录
2026-11-16：新增 IKPyKinematics、扩展 IKAdapter 显式单轴验证、增加7项测试和 check_ikpy_integration.py。
验收：unittest discover 共22项exit 0；额外40个模型内完整姿态/单轴目标通过exit 0；reuse_demo exit 0。
结果：data/ikpy_integration_report.json；说明：docs/architecture/IKPY_INTEGRATION.md。所有验证为离线，无实物精度或棋盘覆盖结论。
有界新上下文复核完成：未发现具体缺陷或未声明约定；同家族provisional，只读核查，未独立重跑测试。原始结果见 IKPY_REVIEW.json，不作为硬件正确性证明。
