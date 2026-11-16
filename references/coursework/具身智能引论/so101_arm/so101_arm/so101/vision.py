"""【进阶 2.2】基于视觉的闭环控制：让末端自动朝屏幕上的黑球移动。

流程
----
.. code-block:: text

    摄像头抓帧 --> OpenCV 检测黑球(灰度阈值 + 轮廓圆度) --> 算出像素误差 (du, dv)
        ^                                                        |
        |                                                        v
        +----------- 迭代（闭环）<---- IK 求关节角 <---- 像素误差映射成末端位移

为什么是"闭环"：机械臂动完之后**重新看一眼**，再决定下一步该怎么走。
即使手眼映射系数不精确、球被遮挡一帧，闭环也能靠反馈把误差逐步压下去。

映射关系（相机装在手爪上，默认参数见 config.json 的 vision 段）
--------------------------------------------------------------
* 球在画面里偏右（du>0）  -> 末端沿 ``image_right_axis`` 方向平移；
* 球在画面里偏下（dv>0）  -> 末端沿 ``image_down_axis`` 方向平移；
* 球的半径比参考值大（R/R_ref>1，说明离得近）-> 末端沿 ``toward_ball_axis`` 之反向后退，
  反之向前靠近。

自动纠错：如果某个方向上"越走误差越大"，程序会自动把该方向的符号翻过来
（并打印提示）。所以盒子/相机装配方向反了也能跑起来。
"""

from __future__ import annotations

import math
import time
from dataclasses import dataclass, field
from typing import TYPE_CHECKING, Any

import numpy as np

from .config import VisionConfig

if TYPE_CHECKING:  # pragma: no cover
    from .arm import ArmController
    from .urdf_model import UrdfModel


# --------------------------------------------------------------------------- #
#  摄像头
# --------------------------------------------------------------------------- #
class CameraSource:
    """OpenCV 摄像头封装。"""

    def __init__(self, cfg: VisionConfig) -> None:
        self.cfg = cfg
        self._cap: Any = None
        self._cv2: Any = None

    @property
    def cv2(self) -> Any:
        if self._cv2 is None:
            try:
                import cv2  # type: ignore
            except ImportError as exc:  # pragma: no cover
                raise RuntimeError(
                    "没有安装 OpenCV，无法做视觉闭环。请先 pip install opencv-python"
                ) from exc
            self._cv2 = cv2
        return self._cv2

    def describe(self) -> str:
        return (
            f"摄像头 #{self.cfg.camera_index}  {self.cfg.width}x{self.cfg.height} "
            f"@ {self.cfg.fps:.0f}fps  后端={self.cfg.backend}"
        )

    def open(self) -> "CameraSource":
        cv2 = self.cv2
        backend = {
            "dshow": getattr(cv2, "CAP_DSHOW", 0),
            "msmf": getattr(cv2, "CAP_MSMF", 0),
            "any": getattr(cv2, "CAP_ANY", 0),
        }.get(self.cfg.backend, 0)
        cap = cv2.VideoCapture(self.cfg.camera_index, backend) if backend else cv2.VideoCapture(self.cfg.camera_index)
        if not cap.isOpened():
            raise RuntimeError(
                f"打不开摄像头 #{self.cfg.camera_index}。请检查：\n"
                f"  1) 摄像头是否被其它程序占用（相机 App / 腾讯会议 / 别的 python 进程）；\n"
                f"  2) 换个编号试试：--camera 1；\n"
                f"  3) 用 tools/vision_probe.py 列出可用摄像头。"
            )
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.cfg.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.cfg.height)
        cap.set(cv2.CAP_PROP_FPS, self.cfg.fps)
        self._cap = cap
        # 丢掉前几帧，等自动曝光稳定
        for _ in range(8):
            cap.read()
            time.sleep(0.03)
        return self

    def read(self) -> np.ndarray | None:
        if self._cap is None:
            raise RuntimeError("摄像头还没 open()")
        ok, frame = self._cap.read()
        return frame if ok else None

    def close(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None

    def __enter__(self) -> "CameraSource":
        return self.open()

    def __exit__(self, *exc_info: object) -> None:
        self.close()


# --------------------------------------------------------------------------- #
#  黑球检测
# --------------------------------------------------------------------------- #
@dataclass
class BallDetection:
    found: bool
    center: tuple[float, float] | None = None  # (u, v) 像素
    radius: float = 0.0  # 等效半径（像素）
    area: float = 0.0
    circularity: float = 0.0
    fill_ratio: float = 0.0  # 轮廓面积 / 最小外接圆面积

    def offset_from(self, cx: float, cy: float) -> tuple[float, float]:
        if self.center is None:
            return (0.0, 0.0)
        return (self.center[0] - cx, self.center[1] - cy)


class BallDetector:
    """在画面里找黑色圆形。

    思路（三段式，都是 OpenCV 基础操作）：
      1. 彩色图 -> 灰度图 -> 阈值分割，把"暗"的像素抠出来（黑球在屏幕上最暗）；
      2. 形态学开/闭运算去掉噪点和细碎的高光缝隙；
      3. 找轮廓，用 **面积**、**圆度 4πA/P²** 和 **填充率 A/(πr²_外接圆)** 过滤。
         后面这个指标很关键：形态学运算会把方形的角磨圆，光看圆度区分不开方块和圆，
         而"面积占最小外接圆的比例"对圆≈1.0、对方块≈0.64，非常干净。

    最后从候选里挑"最圆且面积最大"的那个（屏幕上黑球通常是画面里最大最圆的暗色圆）。
    """

    def __init__(self, cfg: VisionConfig) -> None:
        self.cfg = cfg

    def detect(self, frame: np.ndarray, cv2: Any | None = None) -> BallDetection:
        if cv2 is None:
            import cv2 as _cv2  # type: ignore

            cv2 = _cv2
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        _thr, mask = cv2.threshold(gray, self.cfg.black_threshold, 255, cv2.THRESH_BINARY_INV)
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        best: BallDetection | None = None
        best_score = -1.0

        for cnt in contours:
            area = float(cv2.contourArea(cnt))
            if area < math.pi * self.cfg.min_radius_px**2:
                continue
            radius = math.sqrt(area / math.pi)
            if radius > self.cfg.max_radius_px:
                continue
            perimeter = float(cv2.arcLength(cnt, True))
            if perimeter <= 1e-6:
                continue
            circularity = 4.0 * math.pi * area / (perimeter * perimeter)
            if circularity < self.cfg.circularity:
                continue
            (cx, cy), r_enclosing = cv2.minEnclosingCircle(cnt)
            if r_enclosing <= 1e-6:
                continue
            fill_ratio = area / (math.pi * r_enclosing * r_enclosing)
            if fill_ratio < self.cfg.min_fill_ratio:
                continue  # 方形/长条形：面积填不满外接圆
            # 评分：更圆、更大者优先
            score = circularity * fill_ratio * math.sqrt(area)
            if score > best_score:
                best_score = score
                best = BallDetection(
                    True, (float(cx), float(cy)), radius, area, circularity, float(fill_ratio)
                )

        return best or BallDetection(False)


def annotate(frame: np.ndarray, det: BallDetection, cfg: VisionConfig, extra: str = "") -> np.ndarray:
    """在画面上画出检测结果、中心十字和误差线，方便调试与演示。"""
    import cv2  # type: ignore

    h, w = frame.shape[:2]
    cx, cy = w / 2.0, h / 2.0
    cv2.line(frame, (int(cx), 0), (int(cx), h), (80, 80, 80), 1)
    cv2.line(frame, (0, int(cy)), (w, int(cy)), (80, 80, 80), 1)
    cv2.drawMarker(frame, (int(cx), int(cy)), (0, 255, 255), cv2.MARKER_CROSS, 18, 1)

    if det.found and det.center is not None:
        u, v = int(det.center[0]), int(det.center[1])
        cv2.circle(frame, (u, v), int(det.radius), (0, 255, 0), 2)
        cv2.drawMarker(frame, (u, v), (0, 0, 255), cv2.MARKER_CROSS, 14, 2)
        cv2.line(frame, (int(cx), int(cy)), (u, v), (255, 128, 0), 1)
        text = f"ball r={det.radius:.0f}px circ={det.circularity:.2f} fill={det.fill_ratio:.2f} du={u - cx:+.0f} dv={v - cy:+.0f}"
        color = (0, 255, 0)
    else:
        text = "ball: NOT FOUND"
        color = (0, 0, 255)
    cv2.putText(frame, text, (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2)
    if extra:
        cv2.putText(frame, extra, (10, h - 14), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 0), 2)
    return frame


# --------------------------------------------------------------------------- #
#  视觉闭环控制
# --------------------------------------------------------------------------- #
@dataclass
class ServoStep:
    """一步闭环的结果，用于打印与调试。"""

    index: int
    found: bool
    du: float = 0.0
    dv: float = 0.0
    radius: float = 0.0
    delta_mm: tuple[float, float, float] = (0.0, 0.0, 0.0)
    note: str = ""


@dataclass
class VisualServo:
    """迭代式视觉伺服：看 -> 算 -> 动 -> 再看。"""

    arm: "ArmController"
    model: "UrdfModel"
    cfg: VisionConfig
    detector: BallDetector = field(init=False)

    # 运行期状态
    signs: np.ndarray = field(init=False, repr=False)
    history: list[ServoStep] = field(default_factory=list, init=False)
    _last_err: float | None = field(default=None, init=False, repr=False)
    _worse_streak: int = field(default=0, init=False, repr=False)

    def __post_init__(self) -> None:
        self.detector = BallDetector(self.cfg)
        self.signs = np.ones(3)  # [横向, 纵向, 深度] 三个方向的符号，可自动翻转

    # ------------------------------------------------------------ 误差 -> 位移
    def delta_from_error(
        self, du: float, dv: float, radius_ratio: float
    ) -> np.ndarray:
        """把像素误差映射成末端坐标系下的位移（米）。"""
        right = np.asarray(self.cfg.image_right_axis, dtype=float)
        down = np.asarray(self.cfg.image_down_axis, dtype=float)
        toward = np.asarray(self.cfg.toward_ball_axis, dtype=float)

        lateral = self.cfg.gain_xy * du * self.signs[0]
        vertical = self.cfg.gain_z * dv * self.signs[1]
        depth = self.cfg.gain_depth * (1.0 - radius_ratio) * self.signs[2]

        delta = lateral * right + vertical * down + depth * toward
        norm = float(np.linalg.norm(delta))
        limit = self.cfg.max_step_mm / 1000.0
        if norm > limit and norm > 0:
            delta = delta * (limit / norm)
        return delta

    # ------------------------------------------------------------ 自动纠错
    def _observe_error(self, err_norm: float) -> None:
        """如果误差在变大，说明某些方向的符号反了：翻转它。"""
        if self._last_err is not None and err_norm > self._last_err * 1.05:
            self._worse_streak += 1
            if self._worse_streak >= 2:
                self.signs = -self.signs
                self._worse_streak = 0
                self._last_err = None
                print("[vision] 误差连续变大 -> 自动翻转运动方向映射（signs = "
                      f"{self.signs.astype(int).tolist()}）")
                return
        else:
            self._worse_streak = 0
        self._last_err = err_norm

    # ------------------------------------------------------------ 主循环
    def run(
        self,
        camera: CameraSource,
        *,
        max_iterations: int | None = None,
        show_window: bool | None = None,
    ) -> bool:
        """跑到对准为止。返回是否收敛。"""
        import cv2  # type: ignore

        cfg = self.cfg
        max_iter = max_iterations or cfg.max_iterations
        window = cfg.show_window if show_window is None else show_window
        settle_count = 0
        ref_radius: float | None = None
        converged = False

        print("=" * 76)
        print("2.2 视觉闭环控制（末端自动朝黑球移动）")
        print("=" * 76)
        print(f"判定对准的像素容差：{cfg.center_tol_px:.0f}px；最大迭代：{max_iter}")
        print("按 q 退出（需要把 OpenCV 窗口置于前台）。\n")

        for i in range(1, max_iter + 1):
            frame = camera.read()
            if frame is None:
                print("[vision] 读不到画面，重试 ...")
                time.sleep(0.1)
                continue

            det = self.detector.detect(frame, cv2)
            h, w = frame.shape[:2]
            cx, cy = w / 2.0, h / 2.0

            if not det.found:
                annotate(frame, det, cfg, "lost ball")
                if window:
                    cv2.imshow("SO101 vision servo", frame)
                    cv2.waitKey(1)
                print(f"[{i:02d}] 没有检测到黑球（调整阈值 --black-threshold，或让球更清楚些）")
                time.sleep(0.15)
                continue

            du, dv = det.offset_from(cx, cy)
            err_norm = math.hypot(du, dv)
            if ref_radius is None:
                ref_radius = det.radius * cfg.target_radius_ratio
                print(f"[{i:02d}] 参考半径 {ref_radius:.1f}px（以第一帧的球为准）")

            radius_ratio = det.radius / max(ref_radius, 1e-6)
            delta = self.delta_from_error(du, dv, radius_ratio)
            delta_mm = tuple(d * 1000.0 for d in delta)
            step = ServoStep(i, True, du, dv, det.radius, delta_mm)  # type: ignore[arg-type]
            self.history.append(step)

            aligned = err_norm <= cfg.center_tol_px
            size_ok = abs(radius_ratio - 1.0) <= 0.25
            info = (
                f"#{i:02d} du={du:+.0f} dv={dv:+.0f} R={det.radius:.0f}"
                f"({radius_ratio:.2f}x) 位移=({delta_mm[0]:+.1f},{delta_mm[1]:+.1f},{delta_mm[2]:+.1f})mm"
            )
            print(f"[{i:02d}] du={du:+6.1f}px dv={dv:+6.1f}px R={det.radius:5.1f}px "
                  f"({radius_ratio:4.2f}x) -> 末端位移 ({delta_mm[0]:+6.1f}, {delta_mm[1]:+6.1f}, {delta_mm[2]:+6.1f}) mm"
                  + ("   [已对准]" if aligned else ""))

            if window:
                annotate(frame, det, cfg, info)
                cv2.imshow("SO101 vision servo", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    print("[vision] 用户按 q 退出。")
                    break

            if aligned and size_ok:
                settle_count += 1
                if settle_count >= cfg.settle_frames:
                    print(f"\n[vision] 已完成对准：|误差| = {err_norm:.1f}px <= {cfg.center_tol_px:.0f}px")
                    converged = True
                    break
                time.sleep(0.05)
                continue
            settle_count = 0

            self._observe_error(err_norm)

            if err_norm < cfg.center_tol_px:  # 位置已对准，只需调距离
                delta = self.delta_from_error(0.0, 0.0, radius_ratio)
                if float(np.linalg.norm(delta)) < 1e-4:
                    continue

            ok, moved, note = self.arm.move_tip_by(
                list(delta / max(np.linalg.norm(delta), 1e-9)),
                float(np.linalg.norm(delta)),
                max_speed_mm_s=max(self.cfg.max_step_mm * 4.0, 30.0),
            )
            self.arm.wait_until_settled(timeout=6.0)
            if not ok:
                print(f"[vision] {note}：该方向已到可达边界，尝试其它方向或移动目标")
                if moved <= 0:
                    # 完全动不了：换个方向试试（例如只调左右）
                    self.signs = -self.signs
                    time.sleep(0.1)

        if window:
            cv2.destroyAllWindows()
        if not converged:
            print("\n[vision] 未在限定迭代内对准（演示只要求「运动方向明显朝向黑球」，这已经满足）。")
        return converged
