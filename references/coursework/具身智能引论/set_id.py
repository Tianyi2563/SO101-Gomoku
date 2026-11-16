"""Safely change the ID of one Feetech-style serial-bus servo."""

import argparse
import sys
import time

import serial


DEFAULT_PORT = "COM5"
DEFAULT_BAUD_RATE = 1_000_000
ID_ADDRESS = 0x05
TORQUE_ENABLE_ADDRESS = 0x28
EEPROM_LOCK_ADDRESS = 0x30


def packet_checksum(payload: bytes) -> int:
    return (~sum(payload)) & 0xFF


def make_write_command(servo_id: int, address: int, value: int) -> bytes:
    body = bytes((servo_id, 4, 3, address, value))
    return b"\xff\xff" + body + bytes((packet_checksum(body),))


def make_read_command(servo_id: int, address: int, size: int = 1) -> bytes:
    body = bytes((servo_id, 4, 2, address, size))
    return b"\xff\xff" + body + bytes((packet_checksum(body),))


def read_status_packet(
    ser: serial.Serial, expected_id: int, expected_parameter_count: int
) -> bytes:
    """Read one status packet, resynchronizing on FF FF if needed."""
    deadline = time.monotonic() + (ser.timeout or 0.5)
    previous = None

    while time.monotonic() < deadline:
        byte = ser.read(1)
        if not byte:
            continue
        current = byte[0]
        if previous == 0xFF and current == 0xFF:
            header = ser.read(2)
            if len(header) != 2:
                break
            servo_id, length = header
            tail = ser.read(length)
            if len(tail) != length:
                break

            packet = b"\xff\xff" + header + tail
            if packet_checksum(packet[2:-1]) != packet[-1]:
                previous = None
                continue
            if servo_id != expected_id:
                previous = None
                continue
            if length != expected_parameter_count + 2:
                # This also filters a locally echoed instruction packet.
                previous = None
                continue
            if tail[0] != 0:
                raise RuntimeError(
                    f"Servo {servo_id} returned error flags 0x{tail[0]:02X}."
                )
            return packet
        previous = current

    raise TimeoutError(f"No valid response from servo ID {expected_id}.")


def read_register(ser: serial.Serial, servo_id: int, address: int) -> int:
    command = make_read_command(servo_id, address)
    ser.reset_input_buffer()
    ser.write(command)
    ser.flush()
    reply = read_status_packet(ser, servo_id, expected_parameter_count=1)
    return reply[5]


def write_register(ser: serial.Serial, servo_id: int, address: int, value: int) -> None:
    command = make_write_command(servo_id, address, value)
    ser.reset_input_buffer()
    ser.write(command)
    ser.flush()

    # Some servos are configured not to acknowledge WRITE instructions. The
    # final READ-back verification below is therefore authoritative.
    try:
        read_status_packet(ser, servo_id, expected_parameter_count=0)
    except TimeoutError:
        pass
    time.sleep(0.1)


def change_id(port: str, baud_rate: int, old_id: int, new_id: int) -> None:
    if not 0 <= old_id <= 253 or not 0 <= new_id <= 253:
        raise ValueError("Servo IDs must be in the range 0..253.")
    if old_id == new_id:
        raise ValueError("The old and new IDs are identical; nothing to change.")

    with serial.Serial(
        port,
        baud_rate,
        bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE,
        stopbits=serial.STOPBITS_ONE,
        timeout=0.4,
        write_timeout=1.0,
    ) as ser:
        time.sleep(0.3)

        current_id = read_register(ser, old_id, ID_ADDRESS)
        if current_id != old_id:
            raise RuntimeError(
                f"Servo {old_id} reports ID {current_id}; refusing to write."
            )

        write_register(ser, old_id, TORQUE_ENABLE_ADDRESS, 0)
        write_register(ser, old_id, EEPROM_LOCK_ADDRESS, 0)
        write_register(ser, old_id, ID_ADDRESS, new_id)

        try:
            stored_id = read_register(ser, new_id, ID_ADDRESS)
        except Exception:
            # If the ID write did not take effect, leave the old servo locked.
            try:
                write_register(ser, old_id, EEPROM_LOCK_ADDRESS, 1)
            except Exception:
                pass
            raise

        if stored_id != new_id:
            raise RuntimeError(
                f"Read-back verification failed: expected {new_id}, got {stored_id}."
            )
        write_register(ser, new_id, EEPROM_LOCK_ADDRESS, 1)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Change the ID of exactly one connected serial-bus servo."
    )
    parser.add_argument("--port", default=DEFAULT_PORT)
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD_RATE)
    parser.add_argument("--old-id", type=int, default=1)
    parser.add_argument("--new-id", type=int, default=2)
    parser.add_argument(
        "--yes",
        action="store_true",
        help="skip the physical-connection confirmation",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    if not args.yes:
        print(
            "Connect ONLY the servo that should receive the new ID. "
            "Disconnect every other servo from the bus."
        )
        confirmation = input(
            f"Type SECOND to change servo ID {args.old_id} to {args.new_id} "
            f"on {args.port}: "
        )
        if confirmation.strip() != "SECOND":
            raise SystemExit("Cancelled; no data was written.")

    change_id(args.port, args.baud, args.old_id, args.new_id)
    print(f"Verified: servo ID changed from {args.old_id} to {args.new_id}.")


if __name__ == "__main__":
    try:
        main()
    except (serial.SerialException, TimeoutError, RuntimeError, ValueError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
