#!/usr/bin/env python3
"""Hold a bimanual pose under sustained load and log per-servo telemetry.

A2 phase 3 (sustained actuator load). The arms are posed BY HAND with torque
released, the pose is captured, and torque is then enabled at those exact
positions -- so the arms never move under power. Nothing is commanded to a
position it is not already in.

Feetech STS3215 servos report their own internal temperature, current, load and
voltage over the bus, which is what this logs. That is the servo's own sensor
measured against the 70 C limit the firmware protects against, rather than a
housing surface temperature.

Usage:
    python diagnostics/hold_pose_thermal.py --minutes 45 --out results/A2/actuator_load/run_1

While running, type annotations and press ENTER:
    bat 87        battery state of charge, percent
    seat 41.2     external probe reading, degrees C
    note ...      free text

Ctrl-C stops early and releases torque. Torque is ALWAYS released on exit.
"""

import argparse
import csv
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from xlerobot_pro.config import ARMS_PORT, HEAD_PORT
from xlerobot_pro.firmware_limits import ARM_ACCELERATION, ARM_TORQUE_LIMIT

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# Setup writes retry: the bus occasionally returns a corrupted status packet,
# and lerobot's write defaults to a single attempt, so one glitch would abort
# the run before it starts. Telemetry reads in the loop already tolerate this.
RETRY = 5

# Telemetry read once per sample, per bus, via sync_read.
TELEMETRY = ["Present_Position", "Present_Temperature", "Present_Current", "Present_Load"]

FIELDS = ["elapsed_s", "timestamp", "bus", "motor", "position", "temp_c", "current", "load", "target"]


def build_buses():
    """Bus 1 = both arms (1-12). Bus 2 = head (1-2) + wheels (3-5)."""
    arms = {f"left_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES) for j, i in zip(JOINTS, range(1, 7))}
    arms |= {f"right_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES) for j, i in zip(JOINTS, range(7, 13))}
    head = {"head_motor_1": Motor(1, "sts3215", MotorNormMode.DEGREES),
            "head_motor_2": Motor(2, "sts3215", MotorNormMode.DEGREES),
            "base_left_wheel": Motor(3, "sts3215", MotorNormMode.DEGREES),
            "base_back_wheel": Motor(4, "sts3215", MotorNormMode.DEGREES),
            "base_right_wheel": Motor(5, "sts3215", MotorNormMode.DEGREES)}
    return (
        ("arms", FeetechMotorsBus(port=ARMS_PORT, motors=arms)),
        ("head", FeetechMotorsBus(port=HEAD_PORT, motors=head)),
    )


def read_annotations(out_dir: Path, stop: threading.Event) -> None:
    path = out_dir / "annotations.csv"
    with path.open("w", newline="") as handle:
        writer = csv.writer(handle)
        writer.writerow(["timestamp", "kind", "value"])
        handle.flush()
        for raw in sys.stdin:
            if stop.is_set():
                return
            entry = raw.strip()
            if not entry:
                continue
            parts = entry.split(None, 1)
            kind = parts[0].lower()
            value = parts[1] if len(parts) > 1 else ""
            writer.writerow([datetime.now(timezone.utc).isoformat(), kind, value])
            handle.flush()
            print(f"    [logged] {kind} {value}")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--minutes", type=float, default=45.0)
    parser.add_argument("--interval", type=float, default=1.0, help="Sample period, seconds.")
    parser.add_argument("--out", type=Path, required=True, help="Output directory.")
    parser.add_argument("--hold-head", action="store_true",
                        help="Also hold the head pose. Off by default: the head is unloaded "
                             "and holding it adds draw without adding information.")
    args = parser.parse_args()

    out_dir = args.out
    out_dir.mkdir(parents=True, exist_ok=True)

    buses = build_buses()
    for _, bus in buses:
        bus.connect()

    # Arms only: the wheels are continuous-rotation and holding them is meaningless.
    arm_names = [n for n in buses[0][1].motors]
    hold_names = {"arms": arm_names, "head": ["head_motor_1", "head_motor_2"] if args.hold_head else []}

    try:
        # --- 1. release torque so the arms can be posed by hand -------------
        for label, bus in buses:
            if hold_names[label]:
                bus.disable_torque(hold_names[label], num_retry=RETRY)
        print("\n  Torque released. Pose BOTH arms by hand into the working pose.")
        print("  Put the 500 g bottle in the RIGHT gripper and close it by hand.")
        input("  Press ENTER to capture the pose and hold it...")

        # --- 2. capture the pose the operator set ---------------------------
        targets = {}
        for label, bus in buses:
            if hold_names[label]:
                targets[label] = bus.sync_read("Present_Position", hold_names[label], normalize=False)
        print("\n  Captured pose:")
        for label in targets:
            for name, pos in targets[label].items():
                print(f"    {name:24s} {pos}")

        # --- 3. apply firmware limits, then hold exactly where we are -------
        # Enabling torque at the present position means no commanded motion.
        for label, bus in buses:
            for name in hold_names[label]:
                bus.write("Torque_Limit", name, ARM_TORQUE_LIMIT, num_retry=RETRY)
                bus.write("Acceleration", name, ARM_ACCELERATION, num_retry=RETRY)
            if hold_names[label]:
                bus.enable_torque(hold_names[label], num_retry=RETRY)
                bus.sync_write("Goal_Position", targets[label], normalize=False, num_retry=RETRY)
        print(f"\n  Holding. Torque_Limit={ARM_TORQUE_LIMIT}, Acceleration={ARM_ACCELERATION}")
        print(f"  Logging every {args.interval:g}s for {args.minutes:g} min into {out_dir}")
        print("  Type 'bat 87', 'seat 41.2' or 'note ...' then ENTER. Ctrl-C stops early.\n")

        # --- 4. log ---------------------------------------------------------
        stop = threading.Event()
        threading.Thread(target=read_annotations, args=(out_dir, stop), daemon=True).start()

        interrupted = {"flag": False}

        def _sigint(sig, frame):
            interrupted["flag"] = True
        signal.signal(signal.SIGINT, _sigint)

        started = time.monotonic()
        deadline = started + args.minutes * 60.0
        peak = {}
        samples = 0

        with (out_dir / "servo_telemetry.csv").open("w", newline="") as handle:
            writer = csv.DictWriter(handle, fieldnames=FIELDS)
            writer.writeheader()
            while not interrupted["flag"] and time.monotonic() < deadline:
                now = time.monotonic()
                stamp = datetime.now(timezone.utc).isoformat()
                for label, bus in buses:
                    names = list(bus.motors)
                    try:
                        vals = {k: bus.sync_read(k, names, normalize=False) for k in TELEMETRY}
                    except Exception as exc:                      # a dropped packet must not end the run
                        print(f"    [warn] {label} read failed: {type(exc).__name__}")
                        continue
                    for name in names:
                        t = vals["Present_Temperature"][name]
                        writer.writerow({
                            "elapsed_s": f"{now - started:.1f}",
                            "timestamp": stamp,
                            "bus": label,
                            "motor": name,
                            "position": vals["Present_Position"][name],
                            "temp_c": t,
                            "current": vals["Present_Current"][name],
                            "load": vals["Present_Load"][name],
                            "target": targets.get(label, {}).get(name, ""),
                        })
                        if t > peak.get(name, -999):
                            peak[name] = t
                handle.flush()
                samples += 1
                if samples % 60 == 0:
                    hot = max(peak.items(), key=lambda kv: kv[1])
                    print(f"    [{samples/60:.0f} min] hottest: {hot[0]} {hot[1]}C")
                time.sleep(max(0.0, args.interval - (time.monotonic() - now)))

        stop.set()

        # --- 5. report ------------------------------------------------------
        print("\n" + "=" * 60)
        print(f"  samples      {samples}")
        print(f"  duration     {(time.monotonic() - started)/60:.1f} min")
        print("  peak servo temperatures (limit 70 C):")
        for name, t in sorted(peak.items(), key=lambda kv: -kv[1]):
            print(f"    {name:24s} {t:>3} C")
        print(f"  data         {out_dir}")
        print("=" * 60)
        return 0

    finally:
        # Torque is released whatever happens -- normal exit, Ctrl-C, or error.
        print("\n  Releasing torque...")
        for label, bus in buses:
            try:
                if hold_names.get(label):
                    bus.disable_torque(hold_names[label], num_retry=RETRY)
            except Exception as exc:
                print(f"    [warn] could not release {label}: {type(exc).__name__}")
            try:
                bus.disconnect()
            except Exception:
                pass
        print("  Torque released. SUPPORT THE ARMS before they settle.")


if __name__ == "__main__":
    sys.exit(main())
