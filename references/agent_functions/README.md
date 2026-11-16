# 4.2：用外部LLM调用已有SO-101技能

## 当前状态

这不是让Codex代替Agent执行函数。`agent.py`向智谱外部API发送`tools`；模型返回`tool_calls`；本地校验与执行，再以`role=tool`和原`tool_call_id`回传，直到模型总结。原始模型响应和实际执行结果写入每次运行的`runs/<时间>/trace.jsonl`。

控制层强制调用原项目的`open_session(..., mock=True)`，不连接串口，不允许模型切换真机。默认图像来自合成针孔投影场景；检测器、IK、伺服控制器和限位换算使用已有实现。合成场景验证不能代替实机标定、碰撞检测或真实视觉验收。

## 代码盘点与封装范围

原项目：`C:/Users/Lucifer/Documents/ChatGPT/LEROBOT/so101_arm`。可通过`SO101_ARM_DIR`环境变量指定其他路径。原项目文件未修改。

|已有代码|可封装的能力|本版处理|
|---|---|---|
|so101/arm.py + backend.py|状态、关节/末端运动、夹爪、保持、后台伺服|复用Mock后端和原控制器；状态、IK、夹爪、停止已暴露|
|so101/presets.py + presets.json|列举姿态、执行姿态、播放动作序列|list_presets、move_to_pose；序列暂不执行，减少长动作|
|so101/kinematics.py|两连杆解析IK、DLS数值IK、直线运动及无解回溯|通过move_tip间接复用，不让LLM计算关节角|
|so101/urdf_model.py + assets/*.urdf|FK、雅可比、关节限位|状态返回FK末端位置与URDF弧度|
|so101/frames.py|归一化动作值与URDF弧度换算|沿用；不把动作百分数误称为角度|
|so101/trajectory.py|平滑插值、限速、轨迹序列|控制器内部复用，不暴露低层连续指令|
|so101/vision.py|摄像头、黑球检测、误差映射、闭环伺服|detect_object、track_object；单次最多3步后hold|
|so101/voice.py|ASR、录音、中文指令解析、执行器、可选LLM JSON兜底|可作为自然语言输入前端；本版不启动麦克风，也不把JSON兜底当成function calling验收|
|so101/session.py、config.py|配置、会话、模型/映射/后端组装|复用，强制mock=True|
|so101/keys.py|键盘控制|人工交互入口，不直接暴露给LLM|
|app_1_1_presets.py、app_1_2_ik.py|姿态与IK演示|参考入口，调用库而非启动交互程序|
|app_2_1_voice.py、app_2_2_vision.py、app_menu.py|语音、视觉与菜单演示|参考入口，不允许模型随意启动程序|
|tools/check_calib.py、inspect_urdf.py、kin_sanity.py、smoke_arm.py|标定/运动学/控制自检|验收辅助；不自动执行真机wiggle|
|tools/test_voice_parse.py、test_voice_ambiguous.py|语音规则测试|已有能力的测试依据|
|tools/test_vision_detect.py、test_vision_loop.py|合成检测与视觉闭环测试|SyntheticCamera参考已有投影fixture；没有伪造检测结果|
|tools/show_ball_screen.py|显示目标球|现场辅助，不作为机器人执行工具|

当前工作目录其他可参考代码也已盘点：

- `capture_arm.py`：真实拍照、GLM手臂姿态估计、SO-101有限关节运动。可单独包装，但与当前黑球伺服不是同一个任务；本版不把照片估计当成IK或目标检测。
- `comparison30/joint_osc_bridge.py`与其测试：Panda关节目标到OSC映射、FK；机器人和动作语义不同，不用于SO-101。
- `comparison30/server.py`、`client.py`、`run.sh`：VLA推理、LIBERO对照运行，可封装独立的推理/评测工具；不作为本次实体控制工具。
- `official_eval/client.py`、`run.py`：正式评测、下载/校验、GPU服务生命周期；适合独立实验Agent，不向机器人Agent暴露下载或进程控制权。
- `pi05_repro_patch/libero_main.py`：LIBERO执行循环、图像/状态转换、回放；不能直接控制SO-101。
- `pi05_config.py`、`train_smoke_no_save.py`与`pi05_repro/*.sh`：训练配置、训练/推理/资源检查脚本；适合后续实验管理工具，不混进运动工具白名单。
- `analysis_libero10/analyze.py`、`retest.py`：成功率、录像抽帧与逐回合记录；适合离线诊断工具，不用于实时伺服。
- `spreadsheet_work`中的表格脚本及输出/日志/视频：分析或验收材料，不是机器人技能。

## 9个工具

`get_robot_state`、`list_presets`、`move_to_pose`、`move_tip`、`set_gripper`、`detect_object`、`track_object`、`stop_tracking`、`stop_motion`。

`detect_object`只支持黑色圆球。object_id=1代表本会话单目标候选，不具备持久身份、多目标或任意文本定位能力。

`track_object`是有界同步技能，不是持续后台追踪。复用`VisualServo.run`，3步后保持；返回`converged=false`必须如实报告，不能说已经对准。

## 核心代码与Prompt

- `tools.py`：JSON Schema、白名单路由、参数/状态校验、已有技能调用。
- `agent.py`：外部API请求、模型tool_calls、匹配tool_call_id回传、循环次数限制和退出清理。
- `prompt.txt`：实际发送给模型的完整系统Prompt。
- `test_tools.py`：离线测试，脚本化聊天fixture明确不算外部LLM验证。

安全检查不只依赖Prompt：每次运动必须有30秒内的状态查询；每次运动后状态授权失效；参数必须是有限数值；末端单次最多20mm、会话预算50mm、最多8次运动；目标检测15秒过期；丢失目标/参数错误时停止锁存；没有解除停止或真机模式工具。没有任意Python、shell或文件读写工具。

## 运行与验收

在当前工作目录运行，已有环境需要numpy和opencv。若可用，可将`python`替换为合适环境下的`uv run python`。

```powershell
# 不回显输入密钥，不写入文件
python -X utf8 -m agent_functions.agent --ask-key --model glm-4-flash

# 外部模型选择视觉检测/追踪工具
python -X utf8 -m agent_functions.agent --ask-key --model glm-4-flash --task "先查状态，检测黑球并执行有界跟踪，最后停止；如实说明是否收敛及Mock模式。"

# 外部模型选择姿态与夹爪工具
python -X utf8 -m agent_functions.agent --ask-key --model glm-4-flash --task "列出预设，查询状态，执行stand姿态，再查状态，把夹爪设为55，最后停止并说明Mock模式。"

# 不访问API的独立测试
python -X utf8 -m unittest agent_functions.test_tools -v
```

也支持`GLM_API_KEY`或`LLM_API_KEY`环境变量；`LLM_MODEL`和`LLM_API_URL`可更换兼容接口。不要将密钥提交到代码库或粘贴到报告。

验收时展示：Prompt → 9个工具Schema → API返回的`llm_response.tool_calls` → 本地`tool_result` → 回传并由LLM总结。以日志为证，不能以“模型说做完了”为证。`session_cleanup`是程序退出清理，不应冒充模型主动调用停止。
