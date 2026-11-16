import argparse
import time
import serial

PORT = "COM5"
BAUD_RATE = 1_000_000
SERVO_ID = 1


def checksum(body: bytes) -> int:
    return (~sum(body)) & 0xFF


def make_packet(servo_id: int, instruction: int, params: bytes) -> bytes:
    body = bytes([servo_id, len(params) + 2, instruction]) + params
    return b"\xFF\xFF" + body + bytes([checksum(body)])


def move_packet(position: int, run_time: int, speed: int) -> bytes:
    if not 0 <= position <= 4095:
        raise ValueError("position 必须在 0~4095 之间；请按舵机型号调整范围")
    if not 0 <= run_time <= 65535:
        raise ValueError("run_time 必须在 0~65535 之间")
    if not 0 <= speed <= 65535:
        raise ValueError("speed 必须在 0~65535 之间")

    # 从地址 0x2A 开始写：目标位置、运行时间、运行速度，均为小端序。
    params = bytes([0x2A])
    params += position.to_bytes(2, "little")
    params += run_time.to_bytes(2, "little")
    params += speed.to_bytes(2, "little")
    return make_packet(SERVO_ID, 0x03, params)


def main() -> None:
    parser = argparse.ArgumentParser(description="控制飞特/幻尔协议总线舵机")
    parser.add_argument("position", type=int, help="目标位置，例如 400 或 800")
    parser.add_argument("--time", type=int, default=1000, dest="run_time", help="运行时间，默认 1000")
    parser.add_argument("--speed", type=int, default=500, help="运行速度，默认 500")
    args = parser.parse_args()

    packet = move_packet(args.position, args.run_time, args.speed)
    print("发送：", packet.hex(" ").upper())

    with serial.Serial(
        PORT,
        BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=1.0,
        write_timeout=1.0,
    ) as ser:
        time.sleep(0.3)
        ser.reset_input_buffer()
        ser.write(packet)
        ser.flush()
        reply = ser.read(6)

    print(f"收到 {len(reply)} 字节：{reply.hex(' ').upper()}")
    if len(reply) != 6 or reply[:2] != b"\xFF\xFF":
        raise RuntimeError("未收到合法响应；请检查舵机供电、ID、共地和半双工总线适配电路")
    if reply[2] != SERVO_ID:
        raise RuntimeError(f"响应 ID 为 {reply[2]}，预期 {SERVO_ID}")
    if reply[4] != 0:
        raise RuntimeError(f"舵机返回错误码 0x{reply[4]:02X}")
    if reply[5] != checksum(reply[2:5]):
        raise RuntimeError("响应校验和错误")

    print(f"舵机已接受目标位置 {args.position}")


if __name__ == "__main__":
    main()
