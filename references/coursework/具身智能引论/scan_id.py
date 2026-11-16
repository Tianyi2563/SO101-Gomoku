import serial

PORT = "COM5"
BAUD_RATE = 1_000_000


def read_id_command(servo_id):
    # 读取地址 0x05 的一个字节（舵机 ID）
    body = [servo_id, 4, 2, 0x05, 1]
    checksum = (~sum(body)) & 0xFF
    return bytes([0xFF, 0xFF, *body, checksum])


def find_reply(raw, expected_id):
    for start in range(len(raw) - 6):
        reply = raw[start:start + 7]

        if reply[:2] != b"\xff\xff":
            continue
        if reply[2] != expected_id or reply[3] != 3:
            continue
        if reply[-1] != ((~sum(reply[2:-1])) & 0xFF):
            continue
        if reply[5] != expected_id:
            continue

        return True

    return False


found = []

with serial.Serial(PORT, BAUD_RATE, timeout=0.04) as ser:
    for servo_id in range(254):
        ser.reset_input_buffer()
        ser.write(read_id_command(servo_id))
        ser.flush()

        raw = ser.read(32)

        if find_reply(raw, servo_id):
            found.append(servo_id)
            print(f"发现舵机 ID：{servo_id}")

if not found:
    print("没有发现舵机")
else:
    print("扫描完成：", found)