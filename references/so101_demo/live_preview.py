"""SO-101 现场演示：实时预览，点击拍照，GLM 分析后驱动机械臂。"""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
from queue import Empty, Queue
import threading
import tkinter as tk
from tkinter import messagebox, simpledialog, ttk

import cv2
from PIL import Image, ImageTk

from .vision_imitation import (
    analyze_photo,
    choose_camera_index,
    open_camera,
    move_so101,
    parse_model_output,
    read_so101_pose,
    validate_start_pose,
)


PHOTO_DIR = Path(__file__).resolve().parent.parent / "captured_photos"


def saved_api_key() -> str:
    key = os.environ.get("GLM_API_KEY", "").strip()
    if not key and os.name == "nt":
        try:
            import winreg

            with winreg.OpenKey(winreg.HKEY_CURRENT_USER, "Environment") as environment:
                key = str(winreg.QueryValueEx(environment, "GLM_API_KEY")[0]).strip()
        except (FileNotFoundError, OSError):
            pass
    return key


class ArmImitationWindow:
    def __init__(self, root: tk.Tk, camera_index: int, port: str, robot_id: str, dry_run: bool):
        self.root = root
        self.port = port
        self.robot_id = robot_id
        self.dry_run = dry_run
        self.camera = open_camera(camera_index)
        self.camera.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.camera.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.camera_index = camera_index
        self.latest_frame = None
        self.events = Queue()
        self.stop_event = threading.Event()
        self.busy = False
        self.close_when_done = False
        self.api_key = saved_api_key()

        root.title("SO-101 手臂动作模仿")
        root.geometry("1120x750")
        root.minsize(900, 650)
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        outer = ttk.Frame(root, padding=12)
        outer.pack(fill="both", expand=True)
        outer.columnconfigure(0, weight=3)
        outer.columnconfigure(1, weight=2)
        outer.rowconfigure(1, weight=1)

        title = ttk.Label(outer, text="实时画面 → 拍照 → GLM 分析 → SO-101 模仿", font=("Microsoft YaHei UI", 15, "bold"))
        title.grid(row=0, column=0, columnspan=2, sticky="w", pady=(0, 10))

        preview_box = ttk.LabelFrame(outer, text=f"人物摄像头 {camera_index} · 实时预览", padding=6)
        preview_box.grid(row=1, column=0, sticky="nsew", padx=(0, 10))
        self.preview = ttk.Label(preview_box, text="正在打开摄像头…", anchor="center")
        self.preview.pack(fill="both", expand=True)

        side = ttk.Frame(outer)
        side.grid(row=1, column=1, sticky="nsew")
        side.columnconfigure(0, weight=1)
        side.rowconfigure(4, weight=1)

        self.status = tk.StringVar(value="请在画面中摆好手臂动作，然后点击“拍照并模仿”。")
        ttk.Label(side, textvariable=self.status, wraplength=330, font=("Microsoft YaHei UI", 10)).grid(
            row=0, column=0, sticky="ew", pady=(0, 12)
        )

        self.capture_button = ttk.Button(side, text="拍照并模仿", command=self.capture_and_imitate)
        self.capture_button.grid(row=1, column=0, sticky="ew", pady=4)
        self.photo_button = ttk.Button(side, text="只拍照", command=self.capture_only)
        self.photo_button.grid(row=2, column=0, sticky="ew", pady=4)
        self.stop_button = ttk.Button(side, text="停止动作", command=self.stop, state="disabled")
        self.stop_button.grid(row=3, column=0, sticky="ew", pady=(4, 12))

        detail_box = ttk.LabelFrame(side, text="分析结果", padding=6)
        detail_box.grid(row=4, column=0, sticky="nsew")
        detail_box.rowconfigure(0, weight=1)
        detail_box.columnconfigure(0, weight=1)
        self.details = tk.Text(detail_box, wrap="word", font=("Microsoft YaHei UI", 10), state="disabled")
        self.details.grid(row=0, column=0, sticky="nsew")
        scroll = ttk.Scrollbar(detail_box, orient="vertical", command=self.details.yview)
        scroll.grid(row=0, column=1, sticky="ns")
        self.details.configure(yscrollcommand=scroll.set)

        self.photo_path = tk.StringVar(value="尚未拍照")
        ttk.Label(outer, textvariable=self.photo_path, wraplength=1000).grid(
            row=2, column=0, columnspan=2, sticky="w", pady=(10, 0)
        )
        ttk.Label(outer, text="提示：空格键拍照并模仿；模型调用需要网络，画面预览会持续更新。", foreground="#555555").grid(
            row=3, column=0, columnspan=2, sticky="w", pady=(6, 0)
        )
        root.bind("<space>", lambda _event: self.capture_and_imitate())
        self.update_preview()
        self.poll_events()

    def update_preview(self):
        if not self.root.winfo_exists():
            return
        ok, frame = self.camera.read()
        if ok and frame is not None:
            self.latest_frame = cv2.flip(frame, 1)  # 镜像预览与保存的照片保持一致。
            rgb = cv2.cvtColor(self.latest_frame, cv2.COLOR_BGR2RGB)
            image = Image.fromarray(rgb)
            image.thumbnail((800, 600))
            photo = ImageTk.PhotoImage(image)
            self.preview.configure(image=photo, text="")
            self.preview.image = photo
        self.root.after(33, self.update_preview)

    def save_current_frame(self) -> Path:
        if self.latest_frame is None:
            raise RuntimeError("摄像头还没有读到画面。")
        PHOTO_DIR.mkdir(exist_ok=True)
        path = PHOTO_DIR / f"arm_{datetime.now():%Y%m%d_%H%M%S_%f}.jpg"
        ok, encoded = cv2.imencode(".jpg", self.latest_frame.copy())
        if not ok:
            raise RuntimeError("照片编码失败。")
        encoded.tofile(str(path))
        self.photo_path.set(f"本次照片：{path}")
        return path

    def capture_only(self):
        if self.busy:
            return
        try:
            path = self.save_current_frame()
            self.status.set("照片已保存，可以查看画面后点击“拍照并模仿”。")
            self.show_details(f"照片：{path}")
        except RuntimeError as exc:
            self.status.set(str(exc))

    def capture_and_imitate(self):
        if self.busy:
            return
        if not self.api_key:
            self.api_key = simpledialog.askstring("GLM API Key", "请输入 GLM API Key：", show="*", parent=self.root) or ""
            if not self.api_key:
                self.status.set("未输入 API Key，已取消本次拍照。")
                return
        try:
            path = self.save_current_frame()
        except RuntimeError as exc:
            self.status.set(str(exc))
            return
        self.busy = True
        self.stop_event.clear()
        self.capture_button.configure(state="disabled")
        self.photo_button.configure(state="disabled")
        self.stop_button.configure(state="normal")
        self.status.set("已拍照，正在检查机械臂并分析照片…")
        self.show_details(f"照片：{path}\n等待视觉模型分析…")
        threading.Thread(target=self.run_pipeline, args=(path,), daemon=True).start()

    def run_pipeline(self, path: Path):
        try:
            if not self.dry_run:
                pose = read_so101_pose(self.port, self.robot_id)
                validate_start_pose(pose)
            if self.stop_event.is_set():
                raise RuntimeError("操作已停止。")
            self.events.put(("status", "正在调用 GLM 分析手臂姿态…"))
            raw, model = analyze_photo(path, self.api_key)
            description, arm_visible, positions = parse_model_output(raw)
            self.events.put(("result", (model, description, arm_visible, positions)))
            if not arm_visible:
                self.events.put(("status", "照片中没有清晰的人手臂，未驱动机械臂。"))
                return
            if self.stop_event.is_set():
                raise RuntimeError("操作已停止。")
            if self.dry_run:
                self.events.put(("status", "分析完成（演练模式，未驱动机械臂）。"))
                return
            self.events.put(("status", "GLM 已生成关节参数，机械臂正在模仿…"))
            move_so101(positions, self.port, self.robot_id, stop_event=self.stop_event)
            self.events.put(("status", "机械臂已完成模仿。"))
        except Exception as exc:
            self.events.put(("error", str(exc)))
        finally:
            self.events.put(("done", None))

    def show_details(self, content: str):
        self.details.configure(state="normal")
        self.details.delete("1.0", "end")
        self.details.insert("1.0", content)
        self.details.configure(state="disabled")

    def poll_events(self):
        if not self.root.winfo_exists():
            return
        try:
            while True:
                kind, payload = self.events.get_nowait()
                if kind == "status":
                    self.status.set(payload)
                elif kind == "result":
                    model, description, visible, positions = payload
                    content = f"模型：{model}\n动作描述：{description}\n检测到人手臂：{'是' if visible else '否'}\n\n关节目标：\n"
                    content += json.dumps(positions, ensure_ascii=False, indent=2)
                    self.show_details(content)
                elif kind == "error":
                    self.status.set(f"未完成：{payload}")
                elif kind == "done":
                    self.busy = False
                    self.capture_button.configure(state="normal")
                    self.photo_button.configure(state="normal")
                    self.stop_button.configure(state="disabled")
                    if self.close_when_done:
                        self.close_now()
                        return
        except Empty:
            pass
        self.root.after(100, self.poll_events)

    def stop(self):
        if self.busy:
            self.stop_event.set()
            self.status.set("停止请求已发出；机械臂会在下一控制步停止。")

    def on_close(self):
        if self.busy:
            self.stop()
            self.close_when_done = True
            messagebox.showinfo("正在停止", "正在结束本次动作，完成后窗口会自动关闭。", parent=self.root)
        else:
            self.close_now()

    def close_now(self):
        self.camera.release()
        self.root.destroy()


def main():
    parser = argparse.ArgumentParser(description="实时摄像头预览与 SO-101 手臂模仿")
    parser.add_argument("--camera", type=int, help="指定摄像头；默认优先人物镜头 1")
    parser.add_argument("--port", default="COM4", help="SO-101 串口，默认 COM4")
    parser.add_argument("--robot-id", default="my_follower", help="LeRobot 校准 ID")
    parser.add_argument("--dry-run", action="store_true", help="只分析照片，不驱动机械臂")
    args = parser.parse_args()
    camera_index = args.camera if args.camera is not None else choose_camera_index()
    root = tk.Tk()
    try:
        ArmImitationWindow(root, camera_index, args.port, args.robot_id, args.dry_run)
        root.mainloop()
    finally:
        if root.winfo_exists():
            root.destroy()


if __name__ == "__main__":
    main()
