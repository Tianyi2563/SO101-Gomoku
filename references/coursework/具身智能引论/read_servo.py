import time
import serial

PORT = "COM5"
BAUD_RATE = 1_000_000
REQUEST = bytes.fromhex("FF FF 01 04 02 38 02 BE")

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
    ser.write(REQUEST)
    ser.flush()
    reply = ser.read(8)

print(f"收到 {len(reply)} 字节：{reply.hex(' ')}")

if len(reply) != 8:
    raise RuntimeError(
        "未收到完整的 8 字节舵机响应；请检查舵机供电、ID、共地以及单线半双工转换电路"
    )
if reply[:2] != b"\xff\xff":
    raise RuntimeError(f"响应帧头错误：{reply.hex(' ')}")
if reply[2] != 1:
    raise RuntimeError(f"响应来自舵机 ID {reply[2]}，预期 ID 1")

expected_checksum = (~sum(reply[2:7])) & 0xFF
if reply[7] != expected_checksum:
    raise RuntimeError(
        f"校验和错误：收到 0x{reply[7]:02X}，预期 0x{expected_checksum:02X}"
    )

# SCSCL 协议的 16 位数据为高字节在前。
position = (reply[5] << 8) | reply[6]
print("当前位置：", position)
