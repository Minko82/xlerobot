#!/usr/bin/env python3
"""Hold a bimanual pose under sustained load and log per-servo telemetry.

A2 phase 3 (sustained actuator load).

The pose is captured by hand ONCE and saved to --pose-file. Every later run
replays that saved pose, so all loads in a sweep share one geometry -- without
that, the moment arm changes between runs and deflection is not comparable
across loads. Only the right gripper is left free each run, so the operator can
wrap it around the weight while the rest of the pose stays fixed.

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
import json
import signal
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from xlerobot_pro.config import ARMS_PORT, HEAD_PORT
from xlerobot_pro.firmware_limits import (
    ARM_ACCELERATION,
    ARM_TORQUE_LIMIT,
    SERVO_TEMP_CEILING_C,
    TORQUE_RELEASE_SECONDS,
)

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

# Setup writes retry: the bus occasionally returns a corrupted status packet,
# and lerobot's write defaults to a single attempt, so one glitch would abort
# the run before it starts. Telemetry reads in the loop already tolerate this.
RETRY = 5

#: The only joint left free for the operator to set per run.
GRIPPER = "right_gripper"

#: Seconds to let the arms settle after the ramp before recording the baseline.
SETTLE_SECONDS = 3.0


def save_pose(path: Path, targets: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps({k: {n: int(v) for n, v in d.items()} for k, d in targets.items()}, indent=2))


def load_pose(path: Path) -> dict:
    return {k: {n: int(v) for n, v in d.items()} for k, d in json.loads(path.read_text()).items()}


def move_to(buses, hold_names, targets, seconds: float = 6.0, steps: int = 60) -> None:
    """Ramp from the present positions to the target pose.

    Interpolated in small increments rather than a single Goal_Position write,
    so the arms travel at a controlled speed instead of snapping to the target.
    """
    start = {}
    for label, bus in buses:
        if hold_names[label]:
            start[label] = bus.sync_read("Present_Position", hold_names[label], normalize=False)
    for i in range(1, steps + 1):
        frac = i / steps
        for label, bus in buses:
            if not hold_names[label]:
                continue
            step = {n: int(round(start[label][n] + (targets[label][n] - start[label][n]) * frac))
                    for n in hold_names[label]}
            bus.sync_write("Goal_Position", step, normalize=False, num_retry=RETRY)
        time.sleep(seconds / steps)

# Telemetry read once per sample, per bus, via sync_read.
TELEMETRY = ["Present_Position", "Present_Temperature", "Present_Current", "Present_Load"]

FIELDS = ["elapsed_s", "timestamp", "bus", "motor", "position", "temp_c", "current", "load",
          "target", "baseline"]


def release_gently(buses, hold_names, seconds: float = TORQUE_RELEASE_SECONDS, steps: int = 24) -> None:
    """Lower the arms by bleeding torque away instead of cutting it.

    ``disable_torque`` removes holding torque in one step, so a loaded arm
    drops. Stepping ``Torque_Limit`` down instead lets gravity lower it against
    progressively weaker resistance -- the arm descends under its own weight but
    is resisted the whole way, so it settles rather than falls.

    The limit is restored afterwards: it lives in EPROM-backed RAM, and leaving
    it at zero would silently cripple the next run.
    """
    try:
        for i in range(steps - 1, -1, -1):
            limit = int(ARM_TORQUE_LIMIT * i / steps)
            for label, bus in buses:
                for name in hold_names[label]:
                    bus.write("Torque_Limit", name, limit, num_retry=RETRY)
            time.sleep(seconds / steps)
    finally:
        # Restore the configured limit before cutting torque, so the next run
        # starts from the documented value rather than whatever we ended on.
        for label, bus in buses:
            for name in hold_names[label]:
                try:
                    bus.write("Torque_Limit", name, ARM_TORQUE_LIMIT, num_retry=RETRY)
                except Exception:
                    pass


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
    parser.add_argument("--pose-file", type=Path, default=Path("calibration/hold_pose.json"),
                        help="Reference pose. Captured on first use, replayed thereafter "
                             "so every run in a sweep shares one geometry.")
    parser.add_argument("--move-seconds", type=float, default=6.0,
                        help="How long the ramp to the reference pose takes.")
    parser.add_argument("--max-temp", type=float, default=SERVO_TEMP_CEILING_C,
                        help=f"Abort cleanly if any servo exceeds this temperature (C). "
                             f"Default {SERVO_TEMP_CEILING_C} from firmware_limits.py. "
                             "The servos protect themselves at 70 C, so stopping below that "
                             "avoids a firmware trip and records time-to-threshold, which is "
                             "the endurance measurement.")
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
        # Firmware limits go on before any motion.
        for label, bus in buses:
            for name in hold_names[label]:
                bus.write("Torque_Limit", name, ARM_TORQUE_LIMIT, num_retry=RETRY)
                bus.write("Acceleration", name, ARM_ACCELERATION, num_retry=RETRY)

        # Commanded pose (drives Goal_Position) and analysis baseline (where
        # the arms actually settled) are tracked separately -- see below.
        baseline = {"arms": {}, "head": {}}

        if args.pose_file.exists():
            # --- REPLAY: drive to the saved reference pose ------------------
            # Every run in a load sweep must share one geometry, or deflection
            # is not comparable between loads. Replaying a saved pose fixes the
            # moment arm; posing by hand does not.
            targets = load_pose(args.pose_file)
            print(f"\n  Reference pose loaded from {args.pose_file}")
            print("  SUPPORT THE ARMS. They will move to the reference pose.")
            input("  Press ENTER when clear...")
            for label, bus in buses:
                if hold_names[label]:
                    bus.enable_torque(hold_names[label], num_retry=RETRY)
            move_to(buses, hold_names, targets, seconds=args.move_seconds)
            print("  In position.")

            # Free the right gripper only, so the operator can wrap it around
            # the weight without disturbing the rest of the pose.
            arms = buses[0][1]
            arms.disable_torque([GRIPPER], num_retry=RETRY)
            print(f"\n  {GRIPPER} released — everything else is holding.")
            input("  Wrap the gripper around the weight, then press ENTER...")
            grip = arms.sync_read("Present_Position", [GRIPPER], normalize=False)
            targets["arms"][GRIPPER] = grip[GRIPPER]
            arms.enable_torque([GRIPPER], num_retry=RETRY)
            print(f"  {GRIPPER} holding at {grip[GRIPPER]}")

            # Record where the arms actually settle, WITHOUT changing the goal.
            #
            # A position-mode servo applies corrective torque proportional to
            # its position error. Writing the sagged position back as the goal
            # zeroes that error, so the servo stops resisting gravity and drops
            # further -- visibly, by a few mm. The commanded pose must therefore
            # stay fixed; only the analysis baseline moves.
            time.sleep(SETTLE_SECONDS)
            for label, bus in buses:
                if hold_names[label]:
                    settled = bus.sync_read("Present_Position", hold_names[label], normalize=False)
                    for name, pos in settled.items():
                        offset = pos - targets[label][name]
                        if abs(offset) > 3:
                            print(f"    {name:24s} settled {offset:+d} from commanded")
                        baseline[label][name] = pos
        else:
            # --- CAPTURE: pose by hand once, save as the reference ----------
            for label, bus in buses:
                if hold_names[label]:
                    bus.disable_torque(hold_names[label], num_retry=RETRY)
            print("\n  Torque released. Pose BOTH arms by hand into the working pose.")
            print("  This pose will be SAVED and replayed by every later run, so set")
            print("  it deliberately. Leave the gripper open — it is set per run.")
            input("  Press ENTER to capture...")
            targets = {}
            for label, bus in buses:
                if hold_names[label]:
                    targets[label] = bus.sync_read("Present_Position", hold_names[label], normalize=False)
            save_pose(args.pose_file, targets)
            print(f"\n  Reference pose saved to {args.pose_file}")
            for label in targets:
                for name, pos in targets[label].items():
                    print(f"    {name:24s} {pos}")
            for label, bus in buses:
                if hold_names[label]:
                    bus.enable_torque(hold_names[label], num_retry=RETRY)
                    baseline[label] = dict(targets[label])

        # --- hold at the target positions --------------------------------
        for label, bus in buses:
            if hold_names[label]:
                bus.sync_write("Goal_Position", targets[label], normalize=False, num_retry=RETRY)
        print(f"\n  Holding. Torque_Limit={ARM_TORQUE_LIMIT}, Acceleration={ARM_ACCELERATION}")
        print(f"  Logging every {args.interval:g}s for {args.minutes:g} min into {out_dir}")
        print("  Type 'bat 87', 'seat 41.2' or 'note ...' then ENTER. Ctrl-C stops early.\n")

        # --- 4. log ---------------------------------------------------------
        stop = threading.Event()
        threading.Thread(target=read_annotations, args=(out_dir, stop), daemon=True).start()

        interrupted = {"flag": False}
        over = None

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
                            "baseline": baseline.get(label, {}).get(name, ""),
                        })
                        if t > peak.get(name, -999):
                            peak[name] = t
                        if args.max_temp is not None and t and float(t) >= args.max_temp:
                            over = (name, float(t), now - started)
                            interrupted["flag"] = True
                handle.flush()
                samples += 1
                if samples % 60 == 0:
                    hot = max(peak.items(), key=lambda kv: kv[1])
                    print(f"    [{samples/60:.0f} min] hottest: {hot[0]} {hot[1]}C")
                time.sleep(max(0.0, args.interval - (time.monotonic() - now)))

        stop.set()

        # --- 5. report ------------------------------------------------------
        print("\n" + "=" * 60)
        if over:
            print(f"  ABORTED      {over[0]} reached {over[1]:.0f} C "
                  f"at {over[2]/60:.1f} min (limit {args.max_temp:g} C)")
            print(f"  ENDURANCE    {over[2]/60:.1f} min to {args.max_temp:g} C")
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
        print(f"\n  Lowering arms over {TORQUE_RELEASE_SECONDS:g}s (bleeding torque)...")
        try:
            release_gently(buses, hold_names)
        except Exception as exc:
            print(f"    [warn] gentle release failed: {type(exc).__name__} — cutting torque")
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
        print("  Torque released.")


if __name__ == "__main__":
    sys.exit(main())
