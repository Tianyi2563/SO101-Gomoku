# SO-ARM101 机械臂 · 课程验收代码

「具身智能引论」机械臂实验的完整实现，覆盖验收的全部四项：

| 编号 | 任务 | 入口程序 | 关键代码 |
|------|------|----------|----------|
| 1.1 | 多关节连续控制（预设姿态 / 预设动作） | `app_1_1_presets.py` | `so101/presets.py`、`so101/trajectory.py` |
| 1.2 | 逆运动学控制 | `app_1_2_ik.py` | `so101/kinematics.py`、`so101/frames.py` |
| 2.1 | 语音控制 | `app_2_1_voice.py` | `so101/voice.py` |
| 2.2 | 基于视觉的闭环控制 | `app_2_2_vision.py` | `so101/vision.py` |

所有程序都支持 `--mock`：**没接机械臂也能完整演示运行逻辑**，方便提前排练和排查。

---

## 1. 先说结论：三条命令跑起来

```bash
# ① 装依赖（无硬件演示只需 numpy；真机/语音/视觉见 requirements.txt）
pip install -r requirements.txt

# ② 不接硬件，先看整条链路对不对
python app_1_1_presets.py --mock --list      # 看预设姿态 / 动作库
python app_1_2_ik.py      --mock             # 键盘控制末端走直线
python app_2_1_voice.py   --mock --say "向前移动5厘米" --say "挥手"
python app_2_2_vision.py  --probe            # 视觉检测（需要摄像头）

# ③ 接真机（把 config.json 里的串口改对，Windows 一般是 COM3/COM4/COM5）
python app_2_1_voice.py --port COM5
```

---

## 2. 目录结构

```
so101_arm/
├── app_1_1_presets.py      1.1 演示：键盘切换预设姿态 / 播放预设动作
├── app_1_2_ik.py           1.2 演示：键盘控制末端前后左右上下（IK 算关节角）
├── app_2_1_voice.py        2.1 演示：语音 -> 文本 -> 机械臂动作
├── app_2_2_vision.py       2.2 演示：摄像头看黑球 -> 闭环移动
├── config.json             所有可调参数（串口、速度、IK、语音、视觉）
├── presets.json            预设姿态与预设动作（首次运行自动生成）
├── requirements.txt
├── assets/
│   └── so101_new_calib.urdf    官方 URDF（IK 的几何依据）
├── so101/                  ← 库代码
│   ├── urdf_model.py       解析 URDF：关节链、限位、零位、FK、雅可比
│   ├── kinematics.py       解析 IK（两连杆）+ 数值 IK（自适应阻尼 DLS）+ 无解回溯
│   ├── frames.py           动作值(度) <-> URDF 关节角(弧度) 的换算
│   ├── trajectory.py       姿态之间的时间插值（linear / min-jerk / 梯形速度）
│   ├── backend.py          硬件后端：lerobot 真机 / Mock 模拟
│   ├── arm.py              控制器：后台伺服线程、限速、限位、排队执行
│   ├── presets.py          预设姿态 / 预设动作的存取与播放
│   ├── keys.py             键盘输入（msvcrt / pynput）
│   ├── voice.py            ASR + 录音 + 指令解析 + 执行
│   ├── vision.py           摄像头 + 黑球检测 + 视觉闭环
│   ├── config.py           配置加载
│   └── session.py          一键搭好整套运行环境
├── tools/                  ← 自检 / 调试工具（验收前跑一遍）
│   ├── inspect_urdf.py     打印 URDF 关节链、限位、FK 自检
│   ├── kin_sanity.py       运动学自测：解析解回代、IK 精度、直线运动、插值
│   ├── smoke_arm.py        无硬件冒烟测试：姿态切换 / IK 直线 / 平滑性 / 夹爪
│   ├── test_voice_parse.py 语音指令解析自测（不需要麦克风）
│   ├── test_vision_detect.py 黑球检测自测（合成图像，不需要摄像头）
│   └── check_calib.py      标定自检：换算表 + 逐关节 wiggle 测试
└── docs/
    ├── 验收演示脚本.md      照着念就能过验收的流程 + 讲解词
    └── 原理讲解.md          FK/IK/视觉伺服的原理与推导
```

---

## 3. 逆运动学（1.2）是怎么做的

### 3.1 数据从哪来

`assets/so101_new_calib.urdf` 是 SO-ARM100 官方仓库里的 URDF。程序启动时解析出
5 个可动关节（`shoulder_pan / shoulder_lift / elbow_flex / wrist_flex / wrist_roll`）
的原点变换、旋转轴和限位，**不硬编码任何连杆长度**。

实测的关节行程与 URDF 完全吻合（0.5° 以内）：

```
shoulder_pan  ±110°   shoulder_lift ±100°   elbow_flex ±96.83°
wrist_flex    ±95°    wrist_roll  ±160°
```

### 3.2 两条 IK 路线，各司其职

1. **解析解**（`ik_planar_2r`）——就是讲义里"两圆交点 / 余弦定理"的那套公式，
   用来讲解多解性和可达性条件 `|L1-L2| ≤ r ≤ L1+L2`，也用来做单元测试
   （把解回代正运动学，误差 ~1e-17）。
2. **数值解**（`ik_dls`）——真正驱动机械臂用的。5 个自由度、连杆不在同一平面、
   还要满足限位，几何解太繁琐，工程上直接用迭代：

   ```
   e  = p_target - p_current
   J  = ∂p/∂q                              (3xN 位置雅可比)
   Δq = Jᵀ(JJᵀ + λ²I)⁻¹ e + (I - J⁺J)(q_rest - q)·k
        └ 阻尼最小二乘主项 ┘  └ 零空间正则项（腕部不漂移）┘
   q ← clip(q + clamp(Δq), 关节限位)
   ```

   `λ` **自适应**：新解让误差变小就调小 `λ`（趋向牛顿步，收敛快）；误差变大就调大 `λ`
   （趋向梯度下降，稳）。固定 `λ` 会留下约 1.3 mm 的稳态残差，自适应后能到
   **0.1 mm 以内**（`tools/kin_sanity.py` 实测 300/300 收敛）。

### 3.3 够不着怎么办（验收会问）

`move_tip_by()` 里先按整步走；若 IK 失败，就把步长**逐级减半**再试，直到有解——
等于沿着原方向一直走到可达边界为止，符合题目要求的"保留尽可能多的原运动方向"。
距离小于 1 mm 时判定"到头了"，打印：

```
[IK] IK 无解：末端已到达该方向的可达边界（步长缩到 1/32），不再移动。
```

---

## 4. 语音控制（2.1）怎么做的

```
麦克风 ──> sounddevice 录音 + 能量阈值切句 ──> faster-whisper 转文本 ──> 指令解析 ──> 动作
                                                │                        │
                                       终端打印转录文本          ① 本地关键字规则（快、离线）
                                                                ② 匹配不到才问 LLM（可选）
```

* **ASR**：`faster-whisper` 本地离线（默认 `small`，CPU int8），首次运行自动下载模型；
  也可以用 `--asr text` 直接打字当作识别结果，方便调试。

  > **模型已经预置在项目里**（`models/models--Systran--faster-whisper-small/`，约 464 MB），
  > `config.json` 的 `voice.model_dir` 默认就指向它，所以**不需要联网、不需要重新下载**。
  > 如果这份模型不在（比如换了机器），程序会尝试从 HuggingFace 自动下载；
  > 国内网络下载不动时先设镜像再跑：
  >
  > ```powershell
  > $env:HF_ENDPOINT='https://hf-mirror.com'   # PowerShell
  > set HF_ENDPOINT=https://hf-mirror.com      # CMD
  > ```
  >
  > 也可以 `--model-dir D:\path\to\models` 指定已有的模型目录。

  另外程序对 whisper 的"幻觉"做了两层防护：Silero VAD + 噪声段过滤，
  以及一份中文幻觉文本黑名单（静音时 whisper 常"编"出"字幕由 XXX 提供"这类句子），
  识别结果命中就直接当成"没听清"，不会误触发动作。
* **指令覆盖**（`so101/voice.py` 的 `parse_command`）：

  | 类型 | 例子 |
  |------|------|
  | 预设姿态 | 立正 / 抬起 / 向左转 / 向右转 / 前伸 / 向下探 |
  | 预设动作 | 挥手 / 打招呼 / 左右扫一遍 |
  | 末端动作 | 向前移动5厘米 / 往后3厘米 / 向上移动两厘米 / 向左10毫米 |
  | 夹爪 | 张开夹爪 / 夹住 |
  | 其他 | 停下 / 回到中位 |

  距离支持中文数字（"两公分"）和多种单位（mm / cm / m）。
* **LLM 兜底**：`config.json` 里 `voice.use_llm_fallback = true` 且设置了
  `OPENAI_API_KEY` 时，关键字匹配不到会调用大模型返回 JSON 指令。

---

## 5. 视觉闭环（2.2）怎么做的

```
抓帧 ──> 灰度化 + 阈值分割(黑) ──> 形态学去噪 ──> 找轮廓
                                              └─ 按面积 + 圆度 4πA/P² 过滤，取最圆的当黑球
                                                        │
                                  像素误差 (du, dv) + 半径比例 R/R_ref
                                                        │
                          映射成末端位移（左右/上下/前后三个轴）──> IK ──> 移动
                                                        │
                                        ←──── 重新看一眼，闭环 ────┘
```

* **手眼映射**（相机装在末端）：

  ```
  Δp = gain_xy · du · image_right_axis
     + gain_z  · dv · image_down_axis
     + gain_depth · (1 - R/R_ref) · toward_ball_axis
  ```

  并有**自动纠错**：如果连续两步误差反而变大，说明该方向的符号装反了，
  程序自动翻转并打印提示——所以相机装反了也能自己救回来。

* 每一步都限幅（`max_step_mm`）、每步都重新检测，因此对像素噪声、遮挡都鲁棒。
* 单步移动失败（到可达边界）会自动换方向重试，不会卡死。

---

## 6. 标定与参数

`so101/frames.py` 负责把 lerobot 的**动作值（度）**和 URDF 的**关节角（弧度）**对上：

```
q_urdf[rad] = offset + sign × scale × 动作值
```

* `offset` = URDF 关节行程中点（SO101 的 new_calib 约定：标定零点 = 行程中点）；
* `scale` = 行程半宽 / 100，即动作值 ±100 对应整个行程（实测行程与 URDF 吻合）；
* `sign` = 方向，默认全 +1。

**不确定就跑自检**：

```bash
python tools/check_calib.py            # 打印当前姿态的预测末端位置，拿卷尺核对
python tools/check_calib.py --wiggle   # 逐个关节动 +10°，观察方向对不对
```

发现某个关节反向，就在 `config.json` 的 `ik.signs` 里把它改成 `-1`；
动得幅度不对就调 `ik.scales_deg_per_unit`。

---

## 7. 验收前自检清单

```bash
python tools/inspect_urdf.py       # URDF 解析 + FK/雅可比一致性
python tools/kin_sanity.py         # IK 精度、可达性、无解回溯、插值
python tools/smoke_arm.py          # 无硬件跑完整个控制链路
python tools/test_voice_parse.py   # 语音指令解析
python tools/test_vision_detect.py # 黑球检测 + 映射方向
python tools/check_calib.py        # 真机标定换算（接真机时）
```

---

## 8. 常见问题

**Q：提示 `当前 Python 环境没有安装 lerobot`**
A：要么 `pip install lerobot`（真机必须），要么加 `--mock` 先跑逻辑。

**Q：连上真机但机械臂不动 / 报串口错误**
A：① 检查 `config.json` 的 `robot.port`（Windows 在设备管理器看 COM 号）；
② 确认没有别的程序占着串口；③ 确认舵机供电（12V 适配器）已插好。

**Q：键盘按了没反应**
A：Windows 下按键是读**当前终端窗口**的，点一下终端让它获得焦点即可。
想让终端不在前台也能收键，就 `pip install pynput` 然后用 `--keyboard pynput`。

**Q：第一次跑语音识别下载模型失败 / 卡住**
A：模型**已经预置在项目 `models/` 目录里**，正常情况下不需要下载。
如果提示要下载，先设国内镜像再跑：

```powershell
$env:HF_ENDPOINT='https://hf-mirror.com'   # PowerShell
set HF_ENDPOINT=https://hf-mirror.com      # CMD
python app_2_1_voice.py
```

或者用 `--model-dir` 指向别处已有的模型目录。实在下不了就先 `--asr text` 演示。

**Q：语音识别不出东西，或者识别出"字幕…"之类的怪句子**
A：那是 whisper 在噪声/静音上的幻觉，程序已经做了过滤（返回"没听清"）。
现场还是听不清的话：① 离麦克风近一些；② 把 `config.json` 里
`voice.energy_threshold` 从 0.012 调到 0.03；
③ 用 `python app_2_1_voice.py --list-devices` 换一个麦克风（在 `voice.input_device` 里填编号）。

**Q：语音识别不出来**
A：① `python app_2_1_voice.py --list-devices` 确认麦克风编号，必要时在
`config.json` 的 `voice.input_device` 指定；② 环境吵就把 `energy_threshold` 调大
（0.012 → 0.03）；③ 模型太小可以换 `--model medium`（更准但更慢）；
④ 先用 `--asr text` 确认"指令 -> 动作"这一段没问题。

**Q：视觉检测不到黑球**
A：先跑 `python app_2_2_vision.py --probe`，画面里球没被绿圈框住就调
`--black-threshold`（屏幕反光/曝光不足时加大，比如 90~110）；
误检了其它黑色物体就调 `--circularity`（0.62 → 0.75 更严格）。

**Q：视觉闭环越走越远**
A：程序会自动翻转方向并提示。也可以手动改 `config.json` 里的
`vision.image_right_axis` / `image_down_axis` / `toward_ball_axis`。

**Q：末端走直线走一半就停了**
A：那是到了工作空间边界，属于正常行为（题目要求的行为）。终端会打印"IK 无解"和
实际走出的距离。可以先用 `h` 回中位再继续。
