# SO101 五子棋

首期：固定俯拍、固定供料、9×9 自由五子棋。当前已完成基础代码移植与适配，完整实物对弈仍待集成。

- [代码状态与下一步](docs/architecture/IMPLEMENTATION_STATUS.md)
- [复用设计](docs/architecture/REUSE_PLAN.md)
- [上游源码](references/README.md)
- [代码来源和许可证](THIRD_PARTY_NOTICES.md)

离线检查（使用现有 .venv，不连接硬件）：

```powershell
.venv/Scripts/python.exe -m unittest discover -s tests -v
.venv/Scripts/python.exe scripts/demos/reuse_demo.py
```

模拟输出明确使用脚本棋步、模拟 IK/驱动/抓取与合成观测，不代表真机或视觉验收。

IKPy 接入测试（官方 SO101 模型，离线）：

```powershell
.venv/Scripts/python.exe -X utf8 scripts/diagnostics/check_ikpy_integration.py
```

接口、朝向模式与验证边界见 [IKPy 接入说明](docs/architecture/IKPY_INTEGRATION.md)。
