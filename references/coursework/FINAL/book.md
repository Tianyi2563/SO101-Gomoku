可以确认：基础环境与代码链路已经验证通过，但“完整实机验收”还差最后两项现场测试。

已确认：

- `COM4` 能识别 SO-101 的 1–6 号电机。
- 机械臂标定文件存在。
- 摄像头 0、1 均可正常取图。
- 麦克风能被程序识别和录音。
- 语音相关依赖已安装。
- 中文指令解析、预设动作加载、末端逆运动学仿真通过。
- 黑圆检测和闭环纠偏计算通过。
- 5 项自动测试全部通过。

尚未实际执行：

- Whisper 模型首次下载及真人中文语音转录。
- 机械臂通电后的语音动作和视觉闭环运动。

这是出于安全考虑，没有在未知机械臂周边环境下自动启用扭矩。

# SO-101 语音与视觉控制操作指南

## 一、准备工作

1. 给 SO-101 机械臂上电。
2. 将机械臂控制板连接到电脑。
3. 确认机械臂周围没有障碍物。
4. 将机械臂调整到工作范围中间，避免处于关节极限。
5. 打开 PowerShell，进入 LeRobot 项目目录：

```powershell
cd "C:\Users\Lucifer\WPS Cloud\WPSDrive\1744687919\WPS企业云盘\复旦大学\我的企业文档\EVERTHING\LeRobot"
```

后续命令都在该目录执行。

## 二、环境检查

运行：

```powershell
.\.venv\Scripts\python.exe -m so101_arm.environment_check --hardware
```

正常情况下应看到：

```text
OK 必需模块: lerobot
OK 必需模块: numpy
OK 必需模块: cv2
OK 语音模块: sounddevice
OK 语音模块: faster_whisper
OK 标定文件
OK 摄像头
Found IDs: 1, 2, 3, 4, 5, 6
OK: full arm
```

如果电机未找到：

1. 检查机械臂是否通电。
2. 检查 USB 连接。
3. 在 Windows 设备管理器中确认端口是否仍为 `COM4`。
4. 如果端口改变，在后续命令中把 `COM4` 替换为新端口。

## 三、语音控制

### 1. 先进行安全模拟

```powershell
.\.venv\Scripts\python.exe -m so101_arm.voice_control --text "向前移动一厘米" --dry-run
```

预期输出包含：

```text
语音转录: 向前移动一厘米
解析结果: Command(kind='move'...)
机械臂动作: 完成
```

此时不会连接或驱动机械臂。

测试预设动作：

```powershell
.\.venv\Scripts\python.exe -m so101_arm.voice_control --text "开始演示" --dry-run
```

程序应依次打印三个预设动作文件。

### 2. 首次下载语音模型

首次使用建议先在网络正常时运行：

```powershell
.\.venv\Scripts\python.exe -m so101_arm.voice_control --model small --dry-run
```

按 Enter 后说一句：

```text
向前移动一厘米
```

程序会首次下载 Whisper `small` 模型。下载完成后应打印：

```text
语音转录: 向前移动一厘米
```

按提示说“停止”即可退出。

验收当天如果不能联网，必须提前完成这一步。

### 3. 实机语音控制

确保有人守在机械臂旁边，然后运行：

```powershell
.\.venv\Scripts\python.exe .\FINAL\voice_control.py --port COM4 --model small
```

每次按 Enter 开始一段约 4 秒的录音。

支持的指令包括：

```text
向前移动一厘米
向后移动两厘米
向左移动一厘米
向右移动一厘米
向上移动一厘米
向下移动一厘米
回到初始位置
开始演示
停止
急停
```

语音采用本地关键字规则解析，不依赖联网大语言模型，并明确分成三类：

1. **预设姿态**：例如“回到初始姿态”“回零”“复位”，加载单个录制姿态。
2. **预设动作**：例如“开始演示”“执行预设动作”，回放与姿态控制器 B 键相同的 `pose_20260928_203838_demo10s.json` 完整录制轨迹。
3. **末端动作**：例如“向前移动一厘米”“往左走3厘米”，调用已有直线 IK 算法生成并执行关节轨迹。

映射配置位于 `FINAL\voice_commands.json`。验收时不要添加 `--dry-run`，否则只解析和规划、不驱动机械臂。

终端会先打印：

```text
语音转录: ...
解析结果: ...
```

然后机械臂才执行动作。

### 4. 使用已有音频测试

```powershell
.\.venv\Scripts\python.exe -m so101_arm.voice_control --wav demo.wav --dry-run
```

确认转录正确后，再去掉 `--dry-run`。

## 四、视觉闭环控制

### 1. 打开黑圆目标

用浏览器打开：

```text
so101_arm\black_target.html
```

将浏览器全屏显示。让机械臂摄像头正对屏幕，屏幕与机械臂之间不要放置其他物体。

### 2. 确认摄像头编号

本机目前摄像头 0 和摄像头 1 都可以取图。

先测试摄像头 0：

```powershell
.\.venv\Scripts\python.exe -m so101_arm.vision_control --camera 0 --dry-run --max-steps 5
```

如果检测不到屏幕上的黑圆，再测试摄像头 1：

```powershell
.\.venv\Scripts\python.exe -m so101_arm.vision_control --camera 1 --dry-run --max-steps 5
```

检测成功时终端会打印类似：

```text
闭环 1: 圆心=(182,428), 半径=21, 修正[m]=[0.0, 0.005, -0.005]
```

注意：当前安装的是无界面的 OpenCV，不要添加 `--preview`。这不影响图像采集和黑圆检测。

### 3. 确认控制方向

第一次实机测试必须减小步长，并只执行一步：

```powershell
.\.venv\Scripts\python.exe -m so101_arm.vision_control --port COM4 --camera 0 --step-mm 2 --max-steps 1
```

观察末端是否朝黑圆方向运动。

如果左右方向相反：

```powershell
.\.venv\Scripts\python.exe -m so101_arm.vision_control --port COM4 --camera 0 --step-mm 2 --max-steps 1 --x-sign -1
```

如果上下方向相反：

```powershell
.\.venv\Scripts\python.exe -m so101_arm.vision_control --port COM4 --camera 0 --step-mm 2 --max-steps 1 --z-sign -1
```

如果两个方向都相反，同时添加两个参数。

### 4. 运行完整视觉闭环

方向确认正确后，运行：

```powershell
.\.venv\Scripts\python.exe -m so101_arm.vision_control --port COM4 --camera 0 --step-mm 5 --max-steps 30
```

程序的执行过程是：

1. 摄像头采集当前画面。
2. 检测最大的黑色圆形。
3. 计算圆心相对画面中心的偏差。
4. 机械臂移动约 5 mm。
5. 重新采集图像并纠偏。
6. 黑圆居中后，机械臂沿前方逐步接近。
7. 达到目标大小或最大步数后停止。

按 `Q`、空格或 `Esc` 可以停止；紧急情况下也可以按 `Ctrl+C`。

## 五、验收演示顺序

### 语音部分

1. 启动语音控制。
2. 对麦克风说：“向前移动一厘米。”
3. 向助教展示终端中的语音转录文本。
4. 展示机械臂向前移动。
5. 说：“回到初始位置。”
6. 说：“开始演示。”
7. 展示机械臂执行三段预设动作。

### 视觉部分

1. 在屏幕上全屏打开黑圆页面。
2. 将机械臂摄像头朝向屏幕。
3. 启动视觉闭环程序。
4. 向助教展示终端中的圆心、半径和纠偏量。
5. 展示机械臂自动朝黑圆方向逐步运动。
6. 必要时移动浏览器窗口或改变黑圆位置，展示程序会根据新画面继续纠偏。

## 六、安全要求

- 第一次实机测试使用 `--step-mm 2 --max-steps 1`。
- 确认方向后才能增加到 5 mm。
- 机械臂和屏幕之间不得有人手或其他障碍物。
- 始终安排一人在机械臂旁边观察。
- 发现方向错误、碰撞风险或异常抖动时，立即按空格、Esc 或 `Ctrl+C`。
- 不要把摄像头方向参数直接用于不同安装角度的机械臂，必须重新进行一步方向测试。
