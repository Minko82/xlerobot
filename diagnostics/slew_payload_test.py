#!/usr/bin/env python3
"""Slew the loaded arm through a cyclic trajectory and log telemetry.

Serves B1 (payload capacity under motion), arm-slew half. The base does not
move, so this needs no drive capability and is far lower risk than a mobile
test -- but it still applies real inertial load to the grasp.

The trajectory is a joint-space circle: shoulder_pan and shoulder_lift follow
sinusoids 90 degrees out of phase, so the end effector sweeps an arc. Sinusoids
are used because their acceleration is known analytically --

    peak angular acceleration = A * (2*pi/T)^2      [counts/s^2]

-- so an acceleration profile can be dialled in via --amplitude and --period
rather than guessed, then verified against the recorded encoder positions.

SLIP DETECTION: the gripper is commanded to a fixed position throughout. If the
payload slips, the jaws close further and `right_gripper` Present_Position moves
away from its target. That is logged every sample, so slip is detected from the
bus rather than from a high-speed camera with fiducials.

Usage:
    # gentle: +/- 200 counts over 8 s  (~0.0123 counts/ms^2 peak)
    python diagnostics/slew_payload_test.py --cycles 10 --amplitude 200 --period 8 \
        --out results/B1/slew_300g_slow

    # aggressive
    python diagnostics/slew_payload_test.py --cycles 10 --amplitude 400 --period 3 \
        --out results/B1/slew_300g_fast

Ctrl-C stops and releases torque. Torque is ALWAYS released on exit.
"""

import argparse
import csv
import json
import math
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

from xlerobot_pro.config import ARMS_PORT
from xlerobot_pro.firmware_limits import ARM_ACCELERATION, ARM_TORQUE_LIMIT

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
RETRY = 5

#: Joints driven by the trajectory. The rest hold station.
SWEEP = ("right_shoulder_pan", "right_shoulder_lift")
GRIPPER = "right_gripper"

FIELDS = ["elapsed_s", "timestamp", "motor", "commanded", "position", "error",
          "temp_c", "current", "load"]


def build_bus() -> FeetechMotorsBus:
    motors = {f"left_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES) for j, i in zip(JOINTS, range(1, 7))}
    motors |= {f"right_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES) for j, i in zip(JOINTS, range(7, 13))}
    return FeetechMotorsBus(port=ARMS_PORT, motors=motors)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--pose-file", type=Path, default=Path("calibration/hold_pose.json"),
                   help="Reference pose to start from (shared with hold_pose_thermal.py).")
    p.add_argument("--cycles", type=int, default=10)
    p.add_argument("--amplitude", type=int, default=200, help="Sweep amplitude, encoder counts.")
    p.add_argument("--period", type=float, default=8.0, help="Seconds per cycle.")
    p.add_argument("--rate", type=float, default=20.0, help="Command/log rate, Hz.")
    p.add_argument("--move-seconds", type=float, default=6.0)
    args = p.parse_args()

    if not args.pose_file.exists():
        print(f"No reference pose at {args.pose_file}. Run hold_pose_thermal.py first "
              f"to capture one.", file=sys.stderr)
        return 1

    pose = {n: int(v) for n, v in json.loads(args.pose_file.read_text())["arms"].items()}
    args.out.mkdir(parents=True, exist_ok=True)

    # Peak acceleration of A*sin(2*pi*t/T) is A*(2*pi/T)^2.
    omega = 2.0 * math.pi / args.period
    peak_acc = args.amplitude * omega ** 2
    peak_vel = args.amplitude * omega
    print("\n  TRAJECTORY")
    print(f"    joints        {', '.join(SWEEP)} (90 deg out of phase)")
    print(f"    amplitude     +/-{args.amplitude} counts  ({args.amplitude * 0.0879:.1f} deg)")
    print(f"    period        {args.period:g} s   cycles {args.cycles}   duration {args.cycles*args.period:.0f} s")
    print(f"    peak velocity {peak_vel:.0f} counts/s  ({peak_vel*0.0879:.0f} deg/s)")
    print(f"    peak accel    {peak_acc:.0f} counts/s^2  ({peak_acc*0.0879:.0f} deg/s^2)")

    bus = build_bus()
    bus.connect()
    names = list(bus.motors)

    try:
        for n in names:
            bus.write("Torque_Limit", n, ARM_TORQUE_LIMIT, num_retry=RETRY)
            bus.write("Acceleration", n, ARM_ACCELERATION, num_retry=RETRY)

        print("\n  SUPPORT THE ARMS. They will move to the reference pose.")
        input("  Press ENTER when clear...")
        bus.enable_torque(names, num_retry=RETRY)

        start = bus.sync_read("Present_Position", names, normalize=False)
        steps = 60
        for i in range(1, steps + 1):
            f = i / steps
            bus.sync_write("Goal_Position",
                           {n: int(round(start[n] + (pose[n] - start[n]) * f)) for n in names},
                           normalize=False, num_retry=RETRY)
            time.sleep(args.move_seconds / steps)
        print("  In position.")

        bus.disable_torque([GRIPPER], num_retry=RETRY)
        print(f"\n  {GRIPPER} released — everything else is holding.")
        input("  Wrap the gripper around the weight, then press ENTER...")
        grip = bus.sync_read("Present_Position", [GRIPPER], normalize=False)[GRIPPER]
        pose[GRIPPER] = grip
        bus.enable_torque([GRIPPER], num_retry=RETRY)
        print(f"  {GRIPPER} holding at {grip}")

        print(f"\n  Starting {args.cycles} cycles. Ctrl-C aborts.")
        input("  CLEAR THE WORKSPACE, then press ENTER...")

        stop = {"flag": False}
        signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("flag", True))

        t0 = time.monotonic()
        duration = args.cycles * args.period
        period_s = 1.0 / args.rate
        samples = 0
        grip_dev = 0

        with (args.out / "slew_telemetry.csv").open("w", newline="") as fh:
            w = csv.DictWriter(fh, fieldnames=FIELDS)
            w.writeheader()
            while not stop["flag"]:
                t = time.monotonic() - t0
                if t >= duration:
                    break
                cmd = dict(pose)
                cmd[SWEEP[0]] = int(round(pose[SWEEP[0]] + args.amplitude * math.sin(omega * t)))
                cmd[SWEEP[1]] = int(round(pose[SWEEP[1]] + args.amplitude * math.cos(omega * t)))
                try:
                    bus.sync_write("Goal_Position", cmd, normalize=False, num_retry=RETRY)
                    vals = {k: bus.sync_read(k, names, normalize=False)
                            for k in ("Present_Position", "Present_Temperature",
                                      "Present_Current", "Present_Load")}
                except Exception as exc:
                    print(f"    [warn] bus: {type(exc).__name__}")
                    continue
                stamp = datetime.now(timezone.utc).isoformat()
                for n in names:
                    pos = vals["Present_Position"][n]
                    w.writerow({"elapsed_s": f"{t:.3f}", "timestamp": stamp, "motor": n,
                                "commanded": cmd[n], "position": pos, "error": pos - cmd[n],
                                "temp_c": vals["Present_Temperature"][n],
                                "current": vals["Present_Current"][n],
                                "load": vals["Present_Load"][n]})
                grip_dev = max(grip_dev, abs(vals["Present_Position"][GRIPPER] - pose[GRIPPER]))
                fh.flush()
                samples += 1
                if samples % (int(args.rate) * 10) == 0:
                    print(f"    t={t:5.1f}s  gripper deviation {grip_dev:+d} counts")
                time.sleep(max(0.0, period_s - ((time.monotonic() - t0) - t)))

        print("\n" + "=" * 58)
        print(f"  cycles completed   {min(args.cycles, (time.monotonic()-t0)/args.period):.1f}")
        print(f"  samples            {samples}")
        print(f"  peak accel         {peak_acc*0.0879:.0f} deg/s^2 (commanded)")
        print(f"  max gripper dev    {grip_dev} counts ({grip_dev*0.0879:.2f} deg)")
        print(f"  verdict            {'POSSIBLE SLIP - inspect' if grip_dev > 10 else 'no slip detected'}")
        print(f"  data               {args.out}")
        print("=" * 58)
        return 0

    finally:
        print("\n  Releasing torque...")
        try:
            bus.disable_torque(names, num_retry=RETRY)
        except Exception as exc:
            print(f"    [warn] release failed: {type(exc).__name__}")
        try:
            bus.disconnect()
        except Exception:
            pass
        print("  Torque released. SUPPORT THE ARMS.")


if __name__ == "__main__":
    sys.exit(main())
