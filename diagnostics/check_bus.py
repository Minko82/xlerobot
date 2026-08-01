"""Check which motors respond on a serial bus and print their raw positions.

Replaces the old per-port check scripts. Each motor set lives on its own bus,
so the port is picked automatically unless you override it:

    python diagnostics/check_bus.py --motors arm    # Bus 1, /dev/xle_arms
    python diagnostics/check_bus.py --motors head   # Bus 2, /dev/xle_head
    python diagnostics/check_bus.py --motors arm --port /dev/ttyACM0
"""

import argparse

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from xlerobot_pro.config import ARMS_PORT, HEAD_PORT

MOTOR_SETS = {
    "arm": {
        "shoulder_pan": Motor(7, "sts3215", MotorNormMode.DEGREES),
        "shoulder_lift": Motor(8, "sts3215", MotorNormMode.DEGREES),
        "elbow_flex": Motor(9, "sts3215", MotorNormMode.DEGREES),
        "wrist_flex": Motor(10, "sts3215", MotorNormMode.DEGREES),
        "wrist_roll": Motor(11, "sts3215", MotorNormMode.DEGREES),
        "gripper": Motor(12, "sts3215", MotorNormMode.DEGREES),
    },
    "head": {
        "head_motor_1": Motor(1, "sts3215", MotorNormMode.DEGREES),
        "head_motor_2": Motor(2, "sts3215", MotorNormMode.DEGREES),
    },
}


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "--port", default=None, help="Serial port (default: ARMS_PORT for arm, HEAD_PORT for head)"
    )
    parser.add_argument(
        "--motors", choices=sorted(MOTOR_SETS), default="arm", help="Motor set expected on this bus"
    )
    args = parser.parse_args()

    # Each motor set lives on its own bus in the 2-bus wiring.
    if args.port is None:
        args.port = HEAD_PORT if args.motors == "head" else ARMS_PORT

    bus = FeetechMotorsBus(port=args.port, motors=MOTOR_SETS[args.motors])
    try:
        bus.connect()
        print(f"Connected to {args.port} ({args.motors} motors)")
        pos = bus.sync_read("Present_Position", list(bus.motors.keys()))
        for name, val in pos.items():
            print(f"  {name} (ID {bus.motors[name].id}) raw pos: {val}")
    except Exception as e:  # noqa: BLE001 — report any bus failure and exit cleanly
        print(f"Failed to read {args.motors} motors on {args.port}: {e}")
    finally:
        if bus.is_connected:
            bus.disconnect()


if __name__ == "__main__":
    main()
