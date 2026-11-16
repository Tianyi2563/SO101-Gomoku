"""Control two SCSCL servos with normalized values in the range 0.0..1.0."""

import argparse
import sys
import time

import serial


PORT = "COM5"
BAUD_RATE = 1_000_000

# The limits written by set_limits.py.
# servo_id: (minimum_position, maximum_position, logical_angle_range_degrees)
SERVOS = {
    0: (358, 666, 90.0),
    1: (435, 589, 45.0),
}

BROADCAST_ID = 0xFE
WRITE_INSTRUCTION = 0x03
SYNC_WRITE_INSTRUCTION = 0x83
TORQUE_ENABLE_ADDRESS = 0x28
GOAL_POSITION_ADDRESS = 0x2A


def checksum(payload: list[int]) -> int:
    return (~sum(payload)) & 0xFF


def write_command(servo_id: int, address: int, *values: int) -> bytes:
    body = [servo_id, len(values) + 3, WRITE_INSTRUCTION, address, *values]
    return bytes([0xFF, 0xFF, *body, checksum(body)])


def ratio_to_position(servo_id: int, ratio: float) -> int:
    if not 0.0 <= ratio <= 1.0:
        raise ValueError(f"ID {servo_id}: ratio must be between 0 and 1, got {ratio}")
    minimum, maximum, _ = SERVOS[servo_id]
    return round(minimum + ratio * (maximum - minimum))


def uint16_be(value: int) -> tuple[int, int]:
    return value >> 8, value & 0xFF


def sync_position_command(ratios: dict[int, float]) -> tuple[bytes, dict[int, int]]:
    """Build one broadcast SYNC WRITE packet for both goal positions."""
    positions = {
        servo_id: ratio_to_position(servo_id, ratios[servo_id])
        for servo_id in SERVOS
    }

    # SYNC WRITE parameters:
    # [start address, bytes per servo, ID, position H, position L, ...]
    parameters = [GOAL_POSITION_ADDRESS, 2]
    for servo_id in SERVOS:
        high, low = uint16_be(positions[servo_id])
        parameters.extend([servo_id, high, low])

    body = [
        BROADCAST_ID,
        len(parameters) + 2,
        SYNC_WRITE_INSTRUCTION,
        *parameters,
    ]
    return bytes([0xFF, 0xFF, *body, checksum(body)]), positions


def set_torque(ser: serial.Serial, enabled: bool) -> None:
    value = 1 if enabled else 0
    for servo_id in SERVOS:
        ser.reset_input_buffer()
        ser.write(write_command(servo_id, TORQUE_ENABLE_ADDRESS, value))
        ser.flush()
        ser.read(6)
        time.sleep(0.05)


def move(ser: serial.Serial, ratio_0: float, ratio_1: float) -> None:
    ratios = {0: ratio_0, 1: ratio_1}
    command, positions = sync_position_command(ratios)
    ser.reset_input_buffer()
    ser.write(command)
    ser.flush()

    angle_0 = ratio_0 * SERVOS[0][2]
    angle_1 = ratio_1 * SERVOS[1][2]
    print(
        f"ID 0 -> {angle_0:.1f} deg (position {positions[0]}), "
        f"ID 1 -> {angle_1:.1f} deg (position {positions[1]})"
    )


def run_once(ratio_0: float, ratio_1: float) -> None:
    # Validate both values before opening the serial port or enabling torque.
    ratio_to_position(0, ratio_0)
    ratio_to_position(1, ratio_1)

    with serial.Serial(
        PORT,
        BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,
        write_timeout=1.0,
    ) as ser:
        time.sleep(0.3)
        set_torque(ser, True)
        move(ser, ratio_0, ratio_1)


def run_interactive() -> None:
    print("Enter two values from 0 to 1: <ID-0 ratio> <ID-1 ratio>")
    print("Example: 0.5 0.5   |   Enter q to quit and release torque.")

    with serial.Serial(
        PORT,
        BAUD_RATE,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.2,
        write_timeout=1.0,
    ) as ser:
        time.sleep(0.3)
        set_torque(ser, True)
        try:
            while True:
                text = input("ratio_0 ratio_1 > ").strip()
                if text.lower() in {"q", "quit", "exit"}:
                    break
                try:
                    parts = text.split()
                    if len(parts) != 2:
                        raise ValueError("enter exactly two numbers")
                    ratio_0, ratio_1 = map(float, parts)
                    move(ser, ratio_0, ratio_1)
                except ValueError as exc:
                    print(f"Invalid input: {exc}")
        finally:
            print("Releasing servo torque; support the mechanism if necessary.")
            set_torque(ser, False)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Control servo IDs 0 and 1 using normalized values 0..1."
    )
    parser.add_argument("ratio_0", nargs="?", type=float)
    parser.add_argument("ratio_1", nargs="?", type=float)
    args = parser.parse_args()
    if (args.ratio_0 is None) != (args.ratio_1 is None):
        parser.error("provide both ratio_0 and ratio_1, or neither for interactive mode")
    return args


def main() -> None:
    args = parse_args()
    if args.ratio_0 is None:
        run_interactive()
    else:
        run_once(args.ratio_0, args.ratio_1)


if __name__ == "__main__":
    try:
        main()
    except (serial.SerialException, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
