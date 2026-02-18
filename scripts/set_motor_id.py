#!/usr/bin/env python3
"""
Motor ID Configuration

Scans the CAN bus for connected Robstride motors and allows
setting new motor IDs via the CAN protocol.

Uses robstride_dynamics for scanning and raw CAN frames for ID
assignment (works around a bug in robstride_dynamics.write_id).

Usage:
    python scripts/set_motor_id.py                  # Scan and configure
    python scripts/set_motor_id.py --interface can1  # Use different CAN interface
"""

import argparse
import sys

import can
from robstride_dynamics import RobstrideBus

# Robstride CAN protocol constants
MSG_SET_ID = 7
MSG_SAVE_PARAMS = 9
HOST_CAN_ID = 0xFD
SAVE_MAGIC = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])


def scan(channel: str) -> list[int]:
    """Scan the bus and return sorted list of discovered motor IDs."""
    found = RobstrideBus.scan_channel(channel)
    return sorted(found.keys())


def set_id(channel: str, current_id: int, new_id: int) -> bool:
    """Change a motor's CAN ID and save to flash.

    Uses raw CAN frames following the Robstride protocol:
      SetID frame: arb_id = (7 << 24) | (host_id | new_id << 8) << 8 | current_id
      Save frame:  arb_id = (9 << 24) | (host_id << 8) | new_id

    Returns True if the motor responded to both commands.
    """
    bus = can.Bus(channel=channel, interface="socketcan")
    try:
        # Set the new ID
        id_data = HOST_CAN_ID | (new_id << 8)
        arb_id = (MSG_SET_ID << 24) | (id_data << 8) | current_id
        bus.send(can.Message(arbitration_id=arb_id, data=b"\x00" * 8, is_extended_id=True))
        resp = bus.recv(timeout=1.0)
        if not resp:
            print(f"  No response to SetID command (motor {current_id} may be offline).")
            return False

        # Save to non-volatile flash
        save_arb = (MSG_SAVE_PARAMS << 24) | (HOST_CAN_ID << 8) | new_id
        bus.send(can.Message(arbitration_id=save_arb, data=SAVE_MAGIC, is_extended_id=True))
        save_resp = bus.recv(timeout=1.0)
        if not save_resp:
            print("  Warning: no response to save command. Motor may need a power cycle.")
            return False

        return True
    finally:
        bus.shutdown()


def print_motors(motor_ids: list[int]):
    """Print discovered motors."""
    if not motor_ids:
        print("No motors found.")
        return
    print(f"\nFound {len(motor_ids)} motor(s):")
    for motor_id in motor_ids:
        print(f"  ID {motor_id}")


def main():
    parser = argparse.ArgumentParser(description="Robstride motor ID configuration")
    parser.add_argument("--interface", default="can0", help="CAN interface (default: can0)")
    args = parser.parse_args()

    print(f"Scanning {args.interface}...")
    motor_ids = scan(args.interface)
    print_motors(motor_ids)

    if not motor_ids:
        return 1

    print("\nTo set a new ID, enter: <current_id> <new_id>")
    print("Enter 'scan' to rescan, 'quit' to exit.\n")

    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not cmd or cmd == "quit":
            break

        if cmd == "scan":
            print(f"Scanning {args.interface}...")
            motor_ids = scan(args.interface)
            print_motors(motor_ids)
            continue

        parts = cmd.split()
        if len(parts) != 2:
            print("Usage: <current_id> <new_id>")
            continue

        try:
            current_id = int(parts[0])
            new_id = int(parts[1])
        except ValueError:
            print("IDs must be integers.")
            continue

        if current_id not in motor_ids:
            print(f"  Motor ID {current_id} not found on bus. Run 'scan' to refresh.")
            continue

        if not (1 <= new_id <= 127):
            print("  ID must be between 1 and 127.")
            continue

        if new_id in motor_ids:
            print(f"  ID {new_id} is already in use.")
            continue

        if set_id(args.interface, current_id, new_id):
            print(f"  Changed: {current_id} -> {new_id}")

            # Verify
            motor_ids = scan(args.interface)
            if new_id in motor_ids:
                print(f"  Verified: motor responding at ID {new_id}")
            else:
                print("  Warning: could not verify. Motor may need a power cycle.")
        else:
            print(f"  Failed to change ID {current_id} -> {new_id}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
