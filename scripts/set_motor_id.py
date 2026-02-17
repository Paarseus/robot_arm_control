#!/usr/bin/env python3
"""
Motor ID Configuration

Scans the CAN bus for connected RobStride motors and allows
setting new motor IDs using the robstride-dynamics library.

Usage:
    python scripts/set_motor_id.py                  # Scan and configure
    python scripts/set_motor_id.py --interface can1  # Use different CAN interface
"""

import argparse
import sys

from robstride_dynamics import RobstrideBus, Motor, CommunicationType


SAVE_MAGIC = bytes([0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08])


def scan(channel: str) -> dict[int, list[int]]:
    """Scan the bus and return dict of discovered motor IDs."""
    return RobstrideBus.scan_channel(channel)


def print_motors(found: dict[int, list[int]]):
    """Print discovered motors."""
    if not found:
        print("No motors found.")
        return
    print(f"\nFound {len(found)} motor(s):")
    for motor_id in sorted(found.keys()):
        print(f"  ID {motor_id}")


def set_id(channel: str, current_id: int, new_id: int):
    """Change a motor's CAN ID and save to flash."""
    motors = {"target": Motor(id=current_id, model="rs-00")}
    bus = RobstrideBus(channel, motors)
    bus.connect(handshake=False)

    try:
        bus.disable("target")
        bus.write_id("target", new_id)

        # Save to non-volatile memory
        bus.transmit(
            CommunicationType.SAVE_PARAMETERS,
            bus.host_id,
            new_id,
            SAVE_MAGIC
        )
    finally:
        bus.disconnect()


def main():
    parser = argparse.ArgumentParser(description='RobStride motor ID configuration')
    parser.add_argument('--interface', default='can0', help='CAN interface (default: can0)')
    args = parser.parse_args()

    print(f"Scanning {args.interface}...")
    found = scan(args.interface)
    print_motors(found)

    if not found:
        return 1

    motor_ids = sorted(found.keys())

    print("\nTo set a new ID, enter: <current_id> <new_id>")
    print("Enter 'scan' to rescan, 'quit' to exit.\n")

    while True:
        try:
            cmd = input("> ").strip()
        except (EOFError, KeyboardInterrupt):
            print()
            break

        if not cmd or cmd == 'quit':
            break

        if cmd == 'scan':
            found = scan(args.interface)
            print_motors(found)
            motor_ids = sorted(found.keys())
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

        try:
            set_id(args.interface, current_id, new_id)
            print(f"  Set: {current_id} -> {new_id}")

            # Verify
            found = scan(args.interface)
            motor_ids = sorted(found.keys())
            if new_id in motor_ids:
                print(f"  Verified: motor now at ID {new_id}")
            else:
                print("  Warning: could not verify. Motor may need a power cycle.")
        except Exception as e:
            print(f"  Failed: {e}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
