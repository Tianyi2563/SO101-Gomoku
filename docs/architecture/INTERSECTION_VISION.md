# 交点黑白棋识别（第一版）

2026-11-17。已实现交点定位、透视校正、空盘参考差异、黑白棋分类与保守遮挡判断。当前没有真实相机照片，所有验证均为合成图，不能声称实际识别率。

## 棋盘与角色

机械臂默认黑棋先手，人类白棋后手。GameController默认human_color=WHITE，软件对弈命令默认--human white；需要旧角色可显式--human black。

用户已确认实体为9×9方格，即10×10可落子交点；棋局/视觉默认已统一为10×10交点；VisionConfig.intersections控制交点数量。如果实体是9×9方格，应设10，得到10×10交点。当前视觉与Alpha-Beta使用10交点，但现有9×9预训练MCTS不能直接用于10×10；规则/Alpha-Beta可用相应尺寸GameState，controller也必须配置同尺寸。代码不是把棋子放在每格中心。

## 输入与输出

输入：同机位、同分辨率、相同光照曝光的空棋盘照片，以及当前照片。手动提供四个最外侧交点：左上、右上、右下、左下。它们不是棋盘木板四角，也不是棋格中心。首期不自动寻找ArUco或校正镜头畸变；广角畸变明显需预处理。

流程：已知交点四角→透视映射→生成各交点坐标→比较空盘参考→圆形棋子与局部亮度分类→遮挡/异常输出unknown。行向下、列向右，定位点为(row,col)，首点(0,0)，没有半格偏移。

输出：BoardObservation，0空/1黑/-1白/2不确定，每点规则置信分数；另有逐点原因、校正图、标注图与疑似遮挡mask。0.96只是通过当前规则的分数，不是统计校准概率；unknown置信度0，将被现有稳定器/controller拒绝提交。

## 使用

完整合成示例（明确不是实拍）：

```powershell
.venv/Scripts/python.exe -X utf8 scripts/diagnostics/recognize_board.py --config data/samples/SYNTHETIC_vision/config.json --image data/samples/SYNTHETIC_vision/stones_and_occlusion.png --output data/vision-synthetic-result
```

结果：data/vision-synthetic-result/board.json、rectified.png、overlay.png、occlusion.png。标注图绿圈为空，红圈为unknown，黑/白圈为对应棋子；坐标贴在交点旁。

真实图片：复制样例config.json并设置自己的empty_image路径（相对配置文件目录）、calibration_id、outer_intersections_xy和vision.intersections。然后将--image改为实拍图路径。不能沿用示例坐标或把合成空盘当作实物参考。

## 接入现有回合流程

```python
from so101_gomoku.vision.board_detector import BoardObserver, IntersectionDetector
# detector = IntersectionDetector(真实空盘BGR图, 实测四交点, 标定版本)
# observer = BoardObserver(已显式打开的Camera实例, detector)
# controller = GameController(observer, engine, executor, 标定版本)
```

BoardObserver.observe()返回真实capture()时间与递增frame_id，再接入已有多帧确认。它不打开相机、不连接机械臂。离线照片工具JSON中的时间是处理时间，不能拿旧照片的输出直接冒充实时camera给真机controller。

## 适用条件与边界

- 默认近圆棋子直径为格距的0.36~0.86，允许中心偏移0.2格距；可通过VisionConfig调整。单次分类依据参考差异、轮廓圆度和亮度变化。
- 大块/不规则差异、偏离交点的圆、图像覆盖不足或黑白对比不足会输出unknown。也支持传入原图尺寸occlusion_mask，由外部检测器指定不可见区域。
- 这里是“疑似遮挡/异常差异”，不是可识别任何手或机械臂的语义检测。和棋子同大小同形状的异物可能无法区分；小遮挡、镜面反光、圆形机器人部件、参考图错位等均需实拍评估。
- 白棋与背景没有足够可观测对比时无法可靠区分；棋子未引起可见差异时也可能与空位混淆。因此需要选择有明显亮度/颜色差异的棋子背景，验证真实空白棋照片。
- 空盘参考必须确实无棋子；已存在的棋子被写入参考后会被视为背景。移动机位、换棋盘、变曝光后应重新采参考并更新标定版本。
- 现有实现不做漂移补偿或光照自适应，明显光照变化会保守拒识。不要把合成图测试解释为实拍成功率。

## 验证

6项视觉测试通过：空盘、黑白棋和四角交点、相邻棋子、格中心错位不作为棋步、大块/外部mask遮挡、10交点配置、透视校正、亮度异常和默认机械臂黑先。其余全仓库检查结果见data/vision-tests.log。

下一步需要同机位原始照片：空盘、一黑一白、边角/相邻棋子、手或机械臂遮挡。用它们调整阈值并建立实拍回归，之后再评价准确率和误更新风险。

补充：遮挡判定覆盖每个交点半径0.45格距的邻域，不等于整个工作区无人。只处在交点邻域间隙里的小遮挡可能不触发unknown；运动前确认人员/机械臂安全需要独立机制。当前忽略面积小于6像素的差异组件，空位允许局部小于1.5%的差异，这些是固定噪声阈值而非实际标定结果。
