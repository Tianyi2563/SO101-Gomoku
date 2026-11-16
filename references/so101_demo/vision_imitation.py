"""摄像头拍照 -> GLM-4.6V-Flash 识别姿态 -> LeRobot SO-101 模仿。"""

import argparse
import base64
from datetime import datetime
import importlib.util
import json
import math
import os
from pathlib import Path
import sys
import time
import types
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen

import cv2
from .credentials import get_glm_api_key


MODEL_NAME = "glm-4.6v-flash"
FALLBACK_MODEL_NAME = "glm-4v-flash"
API_URL = "https://open.bigmodel.cn/api/paas/v4/chat/completions"
JOINTS = (
    "shoulder_pan",
    "shoulder_lift",
    "elbow_flex",
    "wrist_flex",
    "wrist_roll",
    "gripper",
)
# LeRobot 校准归一化位置。现场手动摆出的姿态只用作 VLM 提示词示例。
# 前五轴距离校准边界预留 5 个单位，避免对端点持续施力。
JOINT_LIMITS = {name: (-95.0, 95.0) for name in JOINTS[:5]}
JOINT_LIMITS["gripper"] = (0.0, 100.0)
CALIBRATION_DIR = (
    Path.home() / ".cache" / "huggingface" / "lerobot" / "calibration" / "robots" / "so101_follower"
)


def make_prompt() -> str:
    joint_details = "\n".join(
        f"- {name}: {meaning}；可输出 {JOINT_LIMITS[name][0]:g}~{JOINT_LIMITS[name][1]:g}。"
        for name, meaning in (
            ("shoulder_pan", "底座左右转向；左右是机械臂自身坐标，不要直接套用画面左右"),
            ("shoulder_lift", "肩关节俯仰，控制上臂相对底座抬起或放下；本机直立约 +6，水平伸直约 +78"),
            ("elbow_flex", "肘关节俯仰，控制前臂相对上臂伸直或弯曲；本机伸直约 -90，展示肌肉式弯曲约 +15"),
            ("wrist_flex", "腕部俯仰，调整夹爪朝向；示例姿态约 +45"),
            ("wrist_roll", "腕部绕前臂轴旋转，调整夹爪翻转；示例姿态约 0"),
            ("gripper", "夹爪开合；本机约 +6 接近闭合，约 +32 可见张开，100 为更大开度"),
        )
    )
    return (
        "只识别照片中真实人物的手臂和手，不要把机械臂或夹爪当成人的手臂。"
        "若画面只有机械臂、夹爪或其他物体，没有真实人物手臂，arm_visible 必须为 false。"
        "请看照片，用中文描述人物手臂姿态，并为 SO-101 机械臂计算六个关节位置。"
        "仅根据可见姿态判断，单张照片不能判断运动方向。"
        "关节值是本机 LeRobot 校准后的归一化位置，不是角度。"
        "机械臂从底座到末端的结构是：底座左右转→肩关节抬放→肘关节屈伸→手腕俯仰→手腕旋转→夹爪开合。"
        "本机的正负方向由下面的现场实测示例定义：\n"
        f"{joint_details}\n"
        "以下数值是同一台机械臂由人手动摆姿势后读取的参考点。"
        "先判定照片最接近哪个姿态，再按差异修改参考点的六个数值。"
        "不要从全零姿态开始推算，也不要输出全零。\n"
        "举手：人的整条手臂向上伸、前臂大致竖直，即使肩部未完整入镜，机械臂也应直立。"
        '{"shoulder_pan":-6,"shoulder_lift":6,"elbow_flex":-95,"wrist_flex":45,"wrist_roll":0,"gripper":6}。\n'
        "水平伸直：人的手臂从肩部向前或向侧面伸出，近似水平且肘部伸直，机械臂向前放平。"
        "向镜头前方伸出时，前臂会因透视而变短，也应按水平伸直判断。"
        '{"shoulder_pan":-6,"shoulder_lift":78,"elbow_flex":-90,"wrist_flex":45,"wrist_roll":0,"gripper":6}。\n'
        "屈肘：人像展示肌肉一样，肘部明显弯成约 90 度，机械臂前臂也要明显折起。"
        '{"shoulder_pan":-5,"shoulder_lift":-92,"elbow_flex":15,"wrist_flex":45,"wrist_roll":0,"gripper":6}。\n'
        "张夹爪：人比划 yes/V 字或手掌张开，保留相应的肩、肘、腕位置，仅把 gripper 从闭合约 6 调到至少 32。"
        "例如举手且手掌张开时，参考位置是"
        '{"shoulder_pan":-6,"shoulder_lift":6,"elbow_flex":-95,"wrist_flex":45,"wrist_roll":0,"gripper":32}。\n'
        "例如屈肘且比 V/剪刀手时，参考位置是"
        '{"shoulder_pan":-5,"shoulder_lift":-92,"elbow_flex":15,"wrist_flex":45,"wrist_roll":0,"gripper":32}。\n'
        "要求：如果照片是明显举手，elbow_flex 应接近 -95，绝不能给 +5；"
        "如果明显水平伸直，shoulder_lift 应接近 +78；如果明显屈肘，elbow_flex 应接近 +15。"
        "夹爪开合可以与其他三个姿态同时出现。若描述含 V、剪刀手或 yes，gripper 必须不小于 32；"
        "若描述是握拳或手指合拢，gripper 应接近 6。"
        "只输出一个 JSON 对象，字段为 description（中文一句话）、arm_visible（布尔值）和 joints（包含上述六个关节名及数字）。"
        "若看不清真实人物手臂则 arm_visible=false；此时不会执行关节指令，但仍要给出六个合法数字。"
    )

if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(encoding="utf-8")
if hasattr(sys.stderr, "reconfigure"):
    sys.stderr.reconfigure(encoding="utf-8")


def open_camera(index: int):
    # Windows 上先尝试 DirectShow，再尝试系统默认后端。
    backends = [cv2.CAP_DSHOW, cv2.CAP_ANY] if sys.platform == "win32" else [cv2.CAP_ANY]
    for backend in backends:
        camera = cv2.VideoCapture(index, backend)
        if camera.isOpened():
            return camera
        camera.release()
    raise RuntimeError(f"无法打开摄像头 {index}。请检查摄像头连接、系统权限及其他占用摄像头的程序。")


def choose_camera_index() -> int:
    """此工作站 1 号为面向人物的镜头，0 号为机械臂镜头。"""
    for index in (1, 0, 2):
        try:
            camera = open_camera(index)
        except RuntimeError:
            continue
        camera.release()
        print(f"使用摄像头 {index}（本机 1 号镜头面向人物）")
        return index
    raise RuntimeError("未找到可用摄像头。")


def capture_photo(camera_index: int | None, wait: bool) -> Path:
    if camera_index is None:
        camera_index = choose_camera_index()
    camera = open_camera(camera_index)
    try:
        # 丢弃刚启动时的若干帧，让自动曝光稳定下来。
        for _ in range(10):
            camera.read()
        if wait:
            input("把手臂摆到镜头前，按回车拍照...")
        ok, frame = camera.read()
        if not ok or frame is None or frame.size == 0:
            raise RuntimeError("摄像头已打开，但未能读取图像。")

        output_dir = Path(__file__).resolve().parent.parent / "captured_photos"
        output_dir.mkdir(exist_ok=True)
        output = output_dir / f"arm_{datetime.now():%Y%m%d_%H%M%S}.jpg"
        encoded_ok, encoded = cv2.imencode(".jpg", frame)
        if not encoded_ok:
            raise RuntimeError("照片编码失败。")
        encoded.tofile(str(output))  # 兼容中文路径
        print(f"拍照成功：{output}")
        print(f"图像尺寸：{frame.shape[1]} x {frame.shape[0]}")
        return output
    finally:
        camera.release()


def analyze_photo(image_path: Path, api_key: str) -> tuple[str, str]:
    if not image_path.is_file():
        raise RuntimeError(f"找不到照片：{image_path}")
    mime_type = {".jpg": "image/jpeg", ".jpeg": "image/jpeg", ".png": "image/png"}.get(
        image_path.suffix.lower()
    )
    if mime_type is None:
        raise RuntimeError("照片必须是 JPG 或 PNG 格式。")

    # 从本地读取刚保存的照片，转成 API 接受的 data URL。
    image_base64 = base64.b64encode(image_path.read_bytes()).decode("ascii")
    data_url = f"data:{mime_type};base64,{image_base64}"
    base_payload = {
        "messages": [
            {
                "role": "user",
                "content": [
                    {"type": "image_url", "image_url": {"url": data_url}},
                    {"type": "text", "text": make_prompt()},
                ],
            }
        ],
    }
    models = tuple(dict.fromkeys((MODEL_NAME, FALLBACK_MODEL_NAME)))
    for model_index, model in enumerate(models):
        payload = {**base_payload, "model": model}
        # 旧视觉模型不需要 thinking 参数，保持与已验证的腕部视觉调用一致。
        if model == MODEL_NAME:
            payload["thinking"] = {"type": "disabled"}
        request = Request(
            API_URL,
            data=json.dumps(payload).encode("utf-8"),
            headers={
                "Authorization": f"Bearer {api_key}",
                "Content-Type": "application/json",
            },
            method="POST",
        )
        for attempt in range(2):
            print(f"正在调用视觉模型 {model}（尝试 {attempt + 1}/2）...", flush=True)
            try:
                with urlopen(request, timeout=90) as response:
                    result = json.load(response)
                content = result["choices"][0]["message"]["content"]
                if not isinstance(content, str) or not content.strip():
                    raise RuntimeError(f"视觉模型 {model} 未返回描述文本。")
                print(f"视觉模型调用成功：{model}", flush=True)
                return content, model
            except HTTPError as exc:
                details = exc.read().decode("utf-8", errors="replace").replace(api_key, "[REDACTED]")[:300]
                if exc.code not in (429, 503):
                    raise RuntimeError(f"视觉模型 {model} 请求失败（HTTP {exc.code}）：{details}") from exc
                if attempt == 0:
                    print(f"视觉模型 {model} 繁忙，2 秒后重试...", flush=True)
                    time.sleep(2)
                    continue
                if model_index + 1 < len(models):
                    print(f"视觉模型 {model} 仍繁忙，自动切换到 {models[model_index + 1]}。", flush=True)
                    break
                raise RuntimeError(f"所有视觉模型均繁忙；最后一次为 {model}（HTTP {exc.code}）：{details}") from exc
            except URLError as exc:
                raise RuntimeError(f"无法连接 API：{exc.reason}") from exc
    raise RuntimeError("没有可用的视觉模型。")


def parse_model_output(content: str) -> tuple[str, bool, dict[str, float]]:
    # 兼容模型偶尔包在 ```json 代码块中的输出。
    start, end = content.find("{"), content.rfind("}")
    if start < 0 or end <= start:
        raise RuntimeError(f"模型未返回 JSON 关节参数：{content}")
    try:
        data = json.loads(content[start : end + 1])
    except json.JSONDecodeError as exc:
        raise RuntimeError(f"模型 JSON 无法解析：{exc}") from exc
    description = data.get("description")
    arm_visible = data.get("arm_visible")
    joints = data.get("joints")
    if not isinstance(description, str) or not description.strip():
        raise RuntimeError("模型缺少动作描述。")
    if not isinstance(arm_visible, bool) or not isinstance(joints, dict):
        raise RuntimeError("模型缺少 arm_visible 或 joints 字段。")

    positions = {}
    for name in JOINTS:
        value = joints.get(name)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise RuntimeError(f"关节 {name} 缺少有效数字。")
        low, high = JOINT_LIMITS[name]
        if not low <= value <= high:
            raise RuntimeError(f"关节 {name} 的数值 {value} 超出机械范围 {low}~{high}。")
        positions[name] = float(value)
    return description.strip(), arm_visible, positions


def make_so101_bus(port: str, robot_id: str):
    calibration_file = CALIBRATION_DIR / f"{robot_id}.json"
    if not calibration_file.is_file():
        raise RuntimeError(f"未找到 SO-101 校准文件：{calibration_file}")
    # LeRobot 0.6.1 的 utils 包入口会连带加载训练用的 torch；
    # 本流程只用其电机模块，缺少 torch 时直接加载该包的硬件子模块。
    if importlib.util.find_spec("torch") is None:
        import lerobot

        utils_package = types.ModuleType("lerobot.utils")
        utils_package.__path__ = [str(Path(lerobot.__file__).parent / "utils")]
        sys.modules["lerobot.utils"] = utils_package
    try:
        from lerobot.motors import Motor, MotorCalibration, MotorNormMode
        from lerobot.motors.feetech import FeetechMotorsBus
    except ImportError as exc:
        raise RuntimeError("缺少 LeRobot 或 Feetech 依赖，请先安装 lerobot[feetech]。") from exc

    calibration_data = json.loads(calibration_file.read_text(encoding="utf-8"))
    if set(calibration_data) != set(JOINTS):
        raise RuntimeError("校准文件的关节名称与 SO-101 六关节不一致。")
    calibration = {name: MotorCalibration(**calibration_data[name]) for name in JOINTS}
    motors = {
        name: Motor(
            calibration[name].id,
            "sts3215",
            MotorNormMode.RANGE_0_100 if name == "gripper" else MotorNormMode.RANGE_M100_100,
        )
        for name in JOINTS
    }
    return FeetechMotorsBus(
        port=port,
        motors=motors,
        calibration=calibration,
    )


def read_so101_pose(port: str, robot_id: str) -> dict[str, float]:
    bus = make_so101_bus(port, robot_id)
    bus.connect()
    try:
        if not bus.is_calibrated:
            raise RuntimeError("机械臂校准与当前电机不一致。")
        return {name: float(value) for name, value in bus.sync_read("Present_Position").items()}
    finally:
        bus.disconnect(disable_torque=False)


def validate_start_pose(positions: dict[str, float]) -> None:
    invalid = {}
    for name in JOINTS:
        value = positions.get(name)
        valid_number = isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
        if not valid_number or not JOINT_LIMITS[name][0] <= value <= JOINT_LIMITS[name][1]:
            invalid[name] = round(value, 2) if valid_number else "无效"
    if invalid:
        raise RuntimeError(f"当前关节超出演示安全范围，先调整姿态再运行：{invalid}")


def move_so101(positions: dict[str, float], port: str, robot_id: str, stop_event=None) -> None:
    if set(positions) != set(JOINTS):
        raise RuntimeError("目标位置必须包含 SO-101 的六个关节。")
    for name, value in positions.items():
        low, high = JOINT_LIMITS[name]
        if not math.isfinite(value) or not low <= value <= high:
            raise RuntimeError(f"关节 {name} 的目标 {value} 超出范围 {low}~{high}。")
    bus = make_so101_bus(port, robot_id)
    from lerobot.motors.feetech import OperatingMode

    bus.connect()
    motion_completed = False
    torque_enabled = False
    try:
        if stop_event is not None and stop_event.is_set():
            raise RuntimeError("操作已停止。")
        if not bus.is_calibrated:
            raise RuntimeError("机械臂校准与当前电机不一致，请先用 LeRobot 重新校准。")
        initial = {name: float(value) for name, value in bus.sync_read("Present_Position").items()}
        if not all(math.isfinite(value) for value in initial.values()):
            raise RuntimeError("无法读取有效的机械臂当前关节位置。")
        validate_start_pose(initial)

        modes = {name: bus.read("Operating_Mode", name) for name in JOINTS}
        if any(mode != OperatingMode.POSITION.value for mode in modes.values()):
            raise RuntimeError(f"电机未处于位置控制模式：{modes}")
        bus.sync_write("Goal_Position", initial)
        torque_enabled = True
        bus.enable_torque()

        # 从当前实测位置匀速插值，每步每轴最多约 1.5 个归一化单位。
        print("本次实际目标：" + json.dumps(positions, ensure_ascii=False))
        max_delta = max(abs(positions[n] - initial[n]) for n in JOINTS)
        steps = max(20, math.ceil(max_delta / 1.5))
        print(f"正在控制 SO-101（{port}），共 {steps} 步...")
        for step in range(1, steps + 1):
            if stop_event is not None and stop_event.is_set():
                raise RuntimeError("机械臂动作已停止。")
            ratio = step / steps
            action = {name: initial[name] + (positions[name] - initial[name]) * ratio for name in JOINTS}
            bus.sync_write("Goal_Position", action)
            time.sleep(0.08)
            if step % 10 == 0:
                observed = bus.sync_read("Present_Position")
                lag = {name: abs(float(observed[name]) - action[name]) for name in JOINTS}
                if max(lag.values()) > 15:
                    raise RuntimeError(f"运动中关节跟踪误差过大，已停止：{lag}")
        actual = {name: float(value) for name, value in bus.sync_read("Present_Position").items()}
        errors = {name: round(abs(actual[name] - positions[name]), 1) for name in JOINTS}
        print("动作后实测位置：" + json.dumps(actual, ensure_ascii=False))
        if max(errors.values()) > 12:
            raise RuntimeError(f"机械臂未到达目标，关节误差：{errors}")
        motion_completed = True
        print("机械臂已到达目标姿态。")
    finally:
        try:
            bus.disconnect(disable_torque=torque_enabled)
        except Exception as exc:
            print(f"关闭舵机扭矩时失败，请检查机械臂供电：{exc}", file=sys.stderr)
            if bus.is_connected:
                bus.disconnect(disable_torque=False)
            if motion_completed:
                raise RuntimeError("机械臂已执行动作，但关闭扭矩失败；请检查供电。") from exc


def main():
    parser = argparse.ArgumentParser(description="摄像头拍照、GLM 视觉分析、SO-101 模仿")
    parser.add_argument("--camera", type=int, help="摄像头编号；本机默认优先人物镜头 1")
    parser.add_argument("--no-wait", action="store_true", help="打开后立即拍照，用于自动测试")
    parser.add_argument("--image", type=Path, help="分析已有照片；不传则先现场拍照")
    parser.add_argument("--capture-only", action="store_true", help="只现场拍照并打印路径，不调用模型或机械臂")
    parser.add_argument("--port", default="COM4", help="SO-101 串口，默认 COM4")
    parser.add_argument("--robot-id", default="my_follower", help="LeRobot 校准 ID")
    parser.add_argument("--dry-run", action="store_true", help="只打印 VLM 结果与目标位置，不连接机械臂")
    parser.add_argument("--show-prompt", action="store_true", help="打印向 VLM 发送的完整提示词")
    parser.add_argument("--read-pose", action="store_true", help="只读取当前六关节位置")
    parser.add_argument("--check-pose", action="store_true", help="只读关节并检查视觉模仿的起始姿态范围")
    args = parser.parse_args()

    if args.show_prompt:
        print(make_prompt())
        return
    if args.read_pose:
        print(json.dumps(read_so101_pose(args.port, args.robot_id), ensure_ascii=False, indent=2))
        return
    if args.check_pose:
        pose = read_so101_pose(args.port, args.robot_id)
        print("当前关节：" + json.dumps(pose, ensure_ascii=False))
        validate_start_pose(pose)
        print("起始姿态检查通过，可以运行视觉模仿。")
        return
    if args.capture_only:
        if args.image:
            parser.error("--capture-only 不能与 --image 同时使用")
        capture_photo(args.camera, not args.no_wait)
        return
    if not args.dry_run:
        validate_start_pose(read_so101_pose(args.port, args.robot_id))

    api_key = get_glm_api_key("请输入 GLM API Key（输入时不会显示）：")
    if not api_key:
        raise RuntimeError("请先设置环境变量 GLM_API_KEY，再运行程序。")
    image_path = args.image if args.image else capture_photo(args.camera, not args.no_wait)
    model_output, used_model = analyze_photo(image_path, api_key)
    print("本次实际使用模型：" + used_model)
    print("模型原始输出：" + model_output)
    description, arm_visible, positions = parse_model_output(model_output)
    print("动作描述：" + description)
    print("目标关节位置：" + json.dumps(positions, ensure_ascii=False))
    if not arm_visible:
        print("照片中未看清手臂，跳过机械臂运动。")
        return
    if args.dry_run:
        print("演练模式：已跳过实体机械臂运动。")
        return
    move_so101(positions, args.port, args.robot_id)


if __name__ == "__main__":
    try:
        main()
    except (RuntimeError, OSError, EOFError) as exc:
        print(f"运行失败：{exc}", file=sys.stderr)
        sys.exit(1)
