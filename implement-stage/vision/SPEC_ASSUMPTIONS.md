# Vision scope and assumptions
2026-11-17; ASK never; draft. 用户已明确单臂机器人黑先、人类白。
V01: 网格尺寸可配置；用户9×9格子含义待回复，暂沿用9×9交点（8×8方格），10×10交点也能识别但9×9预训练AI不兼容。interface/default
V02: 无真实图片提供；以人工指定四个外侧交点+同机位空盘参考图作为首期输入，不实现自动ArUco或镜头畸变标定。semantic/default
V03: 棋子近圆、黑白与空盘亮度有可观差异，直径约0.36~0.86间距；参数可调，超出或遮挡/光变输出unknown；置信度为规则分数，非校准概率。semantic/default
V04: 采用参考图差异组件+交点中心圆形局部区域；不使用棋格中心。大块/不规则差异推为遮挡，外部遮挡mask可补充；不声称语义手/机械臂检测。semantic/default
V05: 默认人类白/机器人黑先；底层controller保留显式换色参数。旧测试显式保留黑方人类场景。interface/user
V06: 阈值只以合成图验证；真实照片识别率/遮挡效果未验证。未知须由稳定器与controller拒绝提交。semantic/default

# Ladder
F0 离线入口与数据接口；F1交点定位/参考差异/分类；F2遮挡+输出可视化；F3角色默认与controller接入；F4合成回归与审阅。
Success: python -m unittest discover -s tests -v; python scripts/diagnostics/recognize_board.py --config ... --image ...
No hardware, no new camera capture. Reuse existing BoardObservation/geometry/controller.
V07(sweep): 逐点遮挡核对半径为0.45格距，交点邻域之间仍有间隙；只在间隙内的遮挡不一定使任何交点unknown，不可作为人手离开整个工作区的安全确认。忽略面积<6像素组件；空位允许<1.5%局部差异，为固定噪声容忍，需实拍调整。

# Outcome
53 tests full suite passed; final margin validation regression6 visual tests passed after fix. Offline CLI synthetic image result generated and overlay visually inspected. Robot black first simulated turn passed. Original photos absent, dimension question pending; synthetic files clearly labeled. Added low-level camera→BoardObservation bridge, no hardware opened. Same-family bounded reviewer found margin type validation issue fixed; ROI gaps and noise thresholds explicitly documented. No claim semantic hand detection or real-photo performance. Review post-fix executor-only.

## 尺寸最终确认（2026-11-17）
用户确认9×9方格=10×10交点；棋局、视觉、网格工具默认10，主对弈CLI默认--size10，机械臂黑先。上游9×9权重不改形状、不冒充适配，Koch引擎需显式--size9仅用于历史参考。原9×9回归用例显式指定size9；新增实际10×10边界和闭环回归。22mm若仍作设计初值则交点跨度198mm，尚无实测确认。
