"""键盘输入：Windows 下用 ``msvcrt`` 非阻塞读键，另外提供可选的 pynput 全局监听。

两个实现：

* :class:`MsvcrtKeyReader` —— 零依赖，**要求终端窗口在前台**（验收演示默认用它）；
* :class:`PynputKeyReader` —— 需要 ``pip install pynput``，全局监听按键，
  即使终端不在前台也能收键（演示时可以一边看机械臂一边按键）。

``KeyReader.poll()`` 返回这一轮读到的所有按键（已归一化成小写字母 / 数字 / 方向键名）。
"""

from __future__ import annotations

import importlib.util
import queue
import sys
import time
from typing import Protocol

# --------------------------------------------------------------------------- #
#  键名归一化
# --------------------------------------------------------------------------- #
_SPECIAL = {
    "\r": "enter",
    "\n": "enter",
    " ": "space",
    "\x1b": "esc",
    "\t": "tab",
    "\x08": "backspace",
    "\x03": "ctrl-c",
}

_ARROWS = {
    "H": "up",
    "P": "down",
    "K": "left",
    "M": "right",
    "G": "home",
    "O": "end",
    "R": "insert",
    "S": "delete",
}


def normalize_key(ch: str) -> str | None:
    """把单个字符归一化成键名；无法识别的返回 None。"""
    if not ch:
        return None
    if ch in _SPECIAL:
        return _SPECIAL[ch]
    if ch.isprintable():
        return ch.lower()
    code = ord(ch)
    if 1 <= code <= 26:  # Ctrl+字母
        return f"ctrl-{chr(96 + code)}"
    return None


# --------------------------------------------------------------------------- #
#  接口
# --------------------------------------------------------------------------- #
class KeyReader(Protocol):
    def poll(self) -> list[str]: ...

    def flush(self) -> None: ...

    def close(self) -> None: ...

    def describe(self) -> str: ...


# --------------------------------------------------------------------------- #
#  Windows：msvcrt
# --------------------------------------------------------------------------- #
class MsvcrtKeyReader:
    """Windows 控制台非阻塞按键读取（终端必须在前台）。"""

    def __init__(self) -> None:
        if importlib.util.find_spec("msvcrt") is None:  # pragma: no cover
            raise RuntimeError("当前环境没有 msvcrt，请改用 PynputKeyReader")

    def describe(self) -> str:
        return "msvcrt（请让终端窗口保持在前台，点一下终端再按键）"

    def poll(self) -> list[str]:
        import msvcrt

        keys: list[str] = []
        while msvcrt.kbhit():
            ch = msvcrt.getwch()
            if ch in ("\x00", "\xe0"):  # 功能键/方向键前缀
                if msvcrt.kbhit():
                    second = msvcrt.getwch()
                    key = _ARROWS.get(second)
                    if key:
                        keys.append(key)
                continue
            key = normalize_key(ch)
            if key:
                keys.append(key)
        return keys

    def flush(self) -> None:
        self.poll()

    def close(self) -> None:
        pass


# --------------------------------------------------------------------------- #
#  可选：pynput 全局监听
# --------------------------------------------------------------------------- #
class PynputKeyReader:
    """基于 pynput 的全局键盘监听，不要求终端在前台。"""

    def __init__(self) -> None:
        from pynput import keyboard  # type: ignore

        self._queue: queue.Queue[str] = queue.Queue()
        self._listener = keyboard.Listener(on_press=self._on_press)
        self._listener.daemon = True
        self._listener.start()

    def _on_press(self, key: object) -> None:
        name: str | None = None
        try:
            from pynput import keyboard  # type: ignore

            if isinstance(key, keyboard.Key):
                name = {
                    keyboard.Key.up: "up",
                    keyboard.Key.down: "down",
                    keyboard.Key.left: "left",
                    keyboard.Key.right: "right",
                    keyboard.Key.space: "space",
                    keyboard.Key.enter: "enter",
                    keyboard.Key.esc: "esc",
                }.get(key)
                if name is None and hasattr(key, "name"):
                    name = f"key-{key.name}"  # type: ignore[attr-defined]
            elif isinstance(key, keyboard.KeyCode) and key.char:
                name = key.char.lower()
        except Exception:  # noqa: BLE001
            name = None
        if name:
            self._queue.put(name)

    def describe(self) -> str:
        return "pynput（全局监听，终端不必在前台）"

    def poll(self) -> list[str]:
        keys: list[str] = []
        while True:
            try:
                keys.append(self._queue.get_nowait())
            except queue.Empty:
                break
        return keys

    def flush(self) -> None:
        self.poll()

    def close(self) -> None:
        try:
            self._listener.stop()
        except Exception:  # noqa: BLE001
            pass


# --------------------------------------------------------------------------- #
#  工厂
# --------------------------------------------------------------------------- #
def make_key_reader(prefer: str = "auto") -> KeyReader:
    """``prefer`` 可选 auto / msvcrt / pynput / none。"""
    prefer = (prefer or "auto").lower()
    if prefer == "none":
        return _NullKeyReader()
    if prefer == "pynput":
        return PynputKeyReader()
    if prefer == "msvcrt":
        return MsvcrtKeyReader()
    # auto：Windows 优先 msvcrt（零依赖、无权限问题）
    if sys.platform.startswith("win"):
        try:
            return MsvcrtKeyReader()
        except RuntimeError:
            pass
    try:
        return PynputKeyReader()
    except Exception:  # noqa: BLE001
        print("[keys] 没有可用的键盘后端：Windows 用 msvcrt，其它平台请 pip install pynput")
        return _NullKeyReader()


class _NullKeyReader:
    """没有键盘后端时的占位实现（配合文本输入模式使用）。"""

    def describe(self) -> str:
        return "无键盘后端（请用文本命令模式）"

    def poll(self) -> list[str]:
        return []

    def flush(self) -> None:
        pass

    def close(self) -> None:
        pass


def wait_for_key(reader: KeyReader, keys: set[str] | None = None, timeout: float | None = None) -> str | None:
    """阻塞等待某个按键（用于"按空格开始录音"这种交互）。"""
    t0 = time.perf_counter()
    while True:
        for k in reader.poll():
            if keys is None or k in keys:
                return k
        if timeout is not None and time.perf_counter() - t0 > timeout:
            return None
        time.sleep(0.01)
