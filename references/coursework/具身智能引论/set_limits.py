"""Set persistent angle limits for two FEETECH SCSCL bus servos."""

import sys
import time

import serial


PORT = "COM5"
BAUD_RATE = 1_000_000

# SCSCL: positions 0..1023 cover approximately 300 degrees.
# Both ranges below are total ranges centered on the neutral position 512.
SERVO_LIMITS = {
    0: (358, 666),  # approximately 90 degrees: 512 +/- 154
    1: (435, 589),  # approximately 45 degrees: 512 +/- 77
}

MIN_ANGLE_LIMIT_ADDRESS = 0x09
TORQUE_ENABLE_ADDRESS = 0x28
EEPROM_LOCK_ADDRESS = 0x30


def write_command(servo_id: int, address: int, *values: int) -> bytes:
    """Build one SCSCL WRITE packet."""
    body = [servo_id, len(values) + 3, 3, address, *values]
    checksum = (~sum(body)) & 0xFF
    return bytes([0xFF, 0xFF, *body, checksum])


def uint16_be(value: int) -> tuple[int, int]:
    """SCSCL sends a 16-bit position high byte first."""
    if not 0 <= value <= 1023:
        raise ValueError(f"Position {value} is outside the SCSCL range 0..1023")
    return value >> 8, value & 0xFF


def set_limits(
    ser: serial.Serial, servo_id: int, minimum: int, maximum: int
) -> None:
    if minimum >= maximum:
        raise ValueError(f"Invalid limits for servo {servo_id}: {minimum}..{maximum}")

    min_high, min_low = uint16_be(minimum)
    max_high, max_low = uint16_be(maximum)
    commands = [
        write_command(servo_id, TORQUE_ENABLE_ADDRESS, 0),
        write_command(servo_id, EEPROM_LOCK_ADDRESS, 0),
        # Addresses 0x09..0x0C are min-high, min-low, max-high, max-low.
        write_command(
            servo_id,
            MIN_ANGLE_LIMIT_ADDRESS,
            min_high,
            min_low,
            max_high,
            max_low,
        ),
        write_command(servo_id, EEPROM_LOCK_ADDRESS, 1),
    ]

    for command in commands:
        ser.reset_input_buffer()
        ser.write(command)
        ser.flush()
        ser.read(6)
        time.sleep(0.1)


def main() -> None:
    print("The servos must have enough mechanical clearance before continuing.")
    for servo_id, (minimum, maximum) in SERVO_LIMITS.items():
        degrees = (maximum - minimum) * 300 / 1024
        print(
            f"ID {servo_id}: {minimum}..{maximum} "
            f"(total range approximately {degrees:.1f} degrees)"
        )

    if input("Type APPLY to write these EEPROM limits: ").strip() != "APPLY":
        raise SystemExit("Cancelled; no data was written.")

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
        for servo_id, limits in SERVO_LIMITS.items():
            set_limits(ser, servo_id, *limits)

    print("Limits written. Torque remains disabled for safe manual inspection.")


if __name__ == "__main__":
    try:
        main()
    except (serial.SerialException, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
