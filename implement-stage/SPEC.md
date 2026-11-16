# SPEC
Target: 将两份参考源码中可复用的能力归档到现有业务目录，完成必要适配与离线验证，明确剩余工作。
Inputs: references/ 两份源码快照、docs/architecture/REUSE_PLAN.md。
Outputs: src/so101_gomoku/ 的可导入模块；tests/ 的离线回归；docs/architecture/IMPLEMENTATION_STATUS.md 的状态与来源；data/sessions/reuse_demo.json 的模拟示例。
Success: .venv/Scripts/python.exe -m unittest discover -s tests -v；.venv/Scripts/python.exe scripts/demos/reuse_demo.py。
Base commit: none (not a git repo; root has no .git)。
Scope: 本次移植基础组件，不自动连接或操作硬件，不宣称视觉准确率或真机可靠性，不实现完整 Alpha-Beta、ArUco 自动定位、网页与完整人机闭环。
