#!/usr/bin/env python3
"""B1 — payload slip under base translation.

The arm grasps a payload and holds a fixed pose while the BASE accelerates. This
is the half of B1 the arm cannot do: the arm's measured dynamic ceiling is about
0.24 m/s^2 at the payload (2.5% of gravity), whereas the base reaches 1.0 m/s^2
acting on the payload directly rather than through a moment arm.

Motion profile per leg: ramp to peak velocity at the target acceleration, dwell,
ramp back down, pause. Legs ALTERNATE DIRECTION so the robot oscillates about its
starting point instead of driving across the room -- with a tether attached, a
base that only ever goes forwards is how cables get torn out.

Realized acceleration comes from wheel odometry over the ramp segment (delta-v
over delta-t), not from differentiating position -- double-differentiating
quantised encoder data at 20 Hz produces noise that swamps the signal, which was
measured and confirmed during the arm-slew work.

Slip is detected from the gripper: the jaws are commanded to a fixed position, so
if the payload moves, `right_gripper` Present_Position departs from its target.
That is a coarse detector -- it catches total loss and deformation but not a rigid
object sliding between parallel jaws. Use the ArUco video method for displacement
in millimetres.

SAFETY: wheels are stopped in a ``finally``, on SIGINT and on SIGTERM, retried and
then VERIFIED by reading speeds back. Arm torque is bled off gradually rather than
cut, so the arm settles instead of dropping.

Usage:
    python diagnostics/b1_base_payload.py --accel 0.25 --cycles 4 \\
        --out results/B1/base_366g_a025
"""

from __future__ import annotations

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
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from xlerobot_pro.config import ARMS_PORT, HEAD_PORT
from xlerobot_pro.firmware_limits import (
    ARM_ACCELERATION,
    ARM_TORQUE_LIMIT,
    TORQUE_RELEASE_SECONDS,
    WHEEL_NECK_ACCELERATION,
    WHEEL_NECK_TORQUE_LIMIT,
)

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]
WHEELS = ("base_left_wheel", "base_back_wheel", "base_right_wheel")
GRIPPER = "right_gripper"
RETRY = 5

#: Arm joints logged each sample. Not all twelve -- two buses at 20 Hz is already
#: a lot of traffic, and these are the ones that carry the payload.
WATCH = ("right_gripper", "right_shoulder_lift", "right_shoulder_pan", "right_elbow_flex")

WHEEL_RADIUS = 0.05
BASE_RADIUS = 0.125
WHEEL_ANGLES_DEG = (240.0, 0.0, 120.0)

GRIPPER_LOAD_CONTACT = 30
GRIPPER_LOAD_WARN = 250
GRIPPER_MAX_STEP = 40

#: Slip threshold, encoder counts of gripper travel.
SLIP_COUNTS = 10

#: Safety ceilings. The protocol's fastest profile is 1.0 m/s^2; the margin above
#: it is small on purpose.
MAX_ACCEL = 1.5      # m/s^2
MAX_PEAK_VEL = 0.6   # m/s
MAX_CYCLES = 20


def degps_to_raw(degps: float) -> int:
    return max(-0x8000, min(0x7FFF, int(round(degps * 4096.0 / 360.0))))


def raw_to_degps(raw: int) -> float:
    return raw * 360.0 / 4096.0


def body_to_wheel_raw(vx: float, vy: float, omega_degps: float) -> dict[str, int]:
    theta = math.radians(omega_degps)
    angles = [math.radians(a - 90.0) for a in WHEEL_ANGLES_DEG]
    degps = [(math.cos(a) * vx + math.sin(a) * vy + BASE_RADIUS * theta)
             / WHEEL_RADIUS * 180.0 / math.pi for a in angles]
    return {n: degps_to_raw(d) for n, d in zip(WHEELS, degps)}


def wheel_raw_to_body(raw: dict[str, int]) -> tuple[float, float, float]:
    angles = [math.radians(a - 90.0) for a in WHEEL_ANGLES_DEG]
    m = [[math.cos(a), math.sin(a), BASE_RADIUS] for a in angles]
    v = [raw_to_degps(raw[n]) * math.pi / 180.0 * WHEEL_RADIUS for n in WHEELS]

    def det3(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))

    d = det3(m)
    if abs(d) < 1e-12:
        return 0.0, 0.0, 0.0
    out = []
    for col in range(3):
        mm = [row[:] for row in m]
        for r in range(3):
            mm[r][col] = v[r]
        out.append(det3(mm) / d)
    return out[0], out[1], math.degrees(out[2])


def build_buses():
    arms = {f"left_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES) for j, i in zip(JOINTS, range(1, 7))}
    arms |= {f"right_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES) for j, i in zip(JOINTS, range(7, 13))}
    head = {"head_motor_1": Motor(1, "sts3215", MotorNormMode.DEGREES),
            "head_motor_2": Motor(2, "sts3215", MotorNormMode.DEGREES),
            "base_left_wheel": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
            "base_back_wheel": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
            "base_right_wheel": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100)}
    return (FeetechMotorsBus(port=ARMS_PORT, motors=arms),
            FeetechMotorsBus(port=HEAD_PORT, motors=head))


def stop_wheels(bus, verify: bool = True) -> bool:
    """Command zero and confirm by reading back. The one failure that matters."""
    for _ in range(4):
        for n in WHEELS:
            try:
                bus.write("Goal_Velocity", n, 0, num_retry=RETRY)
            except Exception:
                pass
        if not verify:
            return True
        time.sleep(0.15)
        try:
            if max(abs(bus.read("Present_Velocity", n, normalize=False, num_retry=RETRY))
                   for n in WHEELS) <= 5:
                return True
        except Exception:
            continue
    return False


def release_gently(bus, names, seconds: float = TORQUE_RELEASE_SECONDS, steps: int = 24) -> None:
    try:
        for i in range(steps - 1, -1, -1):
            limit = int(ARM_TORQUE_LIMIT * i / steps)
            for n in names:
                try:
                    bus.write("Torque_Limit", n, limit, num_retry=RETRY)
                except Exception:
                    pass
            time.sleep(seconds / steps)
    finally:
        for n in names:
            try:
                bus.write("Torque_Limit", n, ARM_TORQUE_LIMIT, num_retry=RETRY)
            except Exception:
                pass


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--pose-file", type=Path, default=Path("calibration/hold_pose.json"))
    p.add_argument("--accel", type=float, default=0.25,
                   help="Target acceleration, m/s^2. Protocol asks for 0.25, 0.5, 1.0.")
    p.add_argument("--peak-vel", type=float, default=0.30, help="Peak velocity, m/s.")
    p.add_argument("--dwell", type=float, default=0.4, help="Seconds held at peak velocity.")
    p.add_argument("--pause", type=float, default=1.5, help="Seconds stationary between legs.")
    p.add_argument("--cycles", type=int, default=4, help="Legs. Direction alternates each leg.")
    p.add_argument("--rate", type=float, default=20.0)
    p.add_argument("--move-seconds", type=float, default=6.0)
    args = p.parse_args()

    if args.accel <= 0 or args.accel > MAX_ACCEL:
        print(f"FATAL: --accel must be in (0, {MAX_ACCEL}] m/s^2.")
        return 1
    if args.peak_vel <= 0 or args.peak_vel > MAX_PEAK_VEL:
        print(f"FATAL: --peak-vel must be in (0, {MAX_PEAK_VEL}] m/s.")
        return 1
    if args.cycles > MAX_CYCLES:
        print(f"FATAL: --cycles capped at {MAX_CYCLES}.")
        return 1
    if not args.pose_file.exists():
        print(f"No reference pose at {args.pose_file}.", file=sys.stderr)
        return 1

    ramp_t = args.peak_vel / args.accel
    leg_dist = args.peak_vel * ramp_t + args.peak_vel * args.dwell   # up + dwell + down
    pose = {n: int(v) for n, v in json.loads(args.pose_file.read_text())["arms"].items()}
    args.out.mkdir(parents=True, exist_ok=True)

    print("\n  B1 — BASE TRANSLATION WITH PAYLOAD")
    print(f"    accel        {args.accel:g} m/s^2   peak vel {args.peak_vel:g} m/s")
    print(f"    ramp time    {ramp_t:.2f} s   dwell {args.dwell:g} s   pause {args.pause:g} s")
    print(f"    legs         {args.cycles} (alternating direction)")
    print(f"    travel/leg   ~{leg_dist*100:.0f} cm, alternating so it stays near home")
    print("\n  THE BASE WILL MOVE AND THE ARM WILL BE CARRYING A LOAD.")
    print("  Clear the floor. Check nothing is tethered.")

    arms, head = build_buses()
    arms.connect()
    head.connect()
    arm_names = list(arms.motors)
    stopping = {"flag": False}
    signal.signal(signal.SIGINT, lambda *_: stopping.__setitem__("flag", True))
    signal.signal(signal.SIGTERM, lambda *_: stopping.__setitem__("flag", True))

    rows = []
    try:
        for n in arm_names:
            arms.write("Torque_Limit", n, ARM_TORQUE_LIMIT, num_retry=RETRY)
            arms.write("Acceleration", n, ARM_ACCELERATION, num_retry=RETRY)
        for n in WHEELS:
            head.write("Operating_Mode", n, OperatingMode.VELOCITY.value, num_retry=RETRY)
            head.write("Torque_Limit", n, WHEEL_NECK_TORQUE_LIMIT, num_retry=RETRY)
            head.write("Acceleration", n, WHEEL_NECK_ACCELERATION, num_retry=RETRY)

        input("\n  Press ENTER to move the arms to the reference pose...")
        arms.enable_torque(arm_names, num_retry=RETRY)
        start = arms.sync_read("Present_Position", arm_names, normalize=False)
        for i in range(1, 61):
            f = i / 60
            arms.sync_write("Goal_Position",
                            {n: int(round(start[n] + (pose[n] - start[n]) * f)) for n in arm_names},
                            normalize=False, num_retry=RETRY)
            time.sleep(args.move_seconds / 60)
        print("  In position.")

        arms.disable_torque([GRIPPER], num_retry=RETRY)
        input(f"\n  {GRIPPER} released — place the object in the jaws, then ENTER...")
        grip = arms.sync_read("Present_Position", [GRIPPER], normalize=False)[GRIPPER]
        pose[GRIPPER] = grip
        arms.enable_torque([GRIPPER], num_retry=RETRY)
        arms.sync_write("Goal_Position", {GRIPPER: pose[GRIPPER]}, normalize=False, num_retry=RETRY)

        print("\n  Squeeze the jaws: signed step (e.g. '+10'), 'r' to re-read, 'ok' when firm.")
        print(f"  Aim for {GRIPPER_LOAD_CONTACT}-{GRIPPER_LOAD_WARN}. Below {GRIPPER_LOAD_CONTACT} the")
        print("  object is only touching, and it WILL fall out once the base accelerates.")
        while True:
            pos = arms.read("Present_Position", GRIPPER, normalize=False, num_retry=RETRY)
            load = arms.read("Present_Load", GRIPPER, num_retry=RETRY)
            flag = "  <-- FIRM" if load >= GRIPPER_LOAD_WARN else (
                "  (NOT GRIPPING)" if load < GRIPPER_LOAD_CONTACT else "")
            print(f"    goal {pose[GRIPPER]:>5}  pos {pos:>5}  load {load:>4}{flag}")
            cmd = input("  > ").strip().lower()
            if cmd in ("ok", "done", ""):
                break
            if cmd == "r":
                continue
            try:
                step = int(cmd)
            except ValueError:
                print("    signed integer, 'r', or 'ok'")
                continue
            step = max(-GRIPPER_MAX_STEP, min(GRIPPER_MAX_STEP, step))
            pose[GRIPPER] += step
            arms.sync_write("Goal_Position", {GRIPPER: pose[GRIPPER]}, normalize=False, num_retry=RETRY)
            time.sleep(0.4)
        grip_ref = arms.read("Present_Position", GRIPPER, normalize=False, num_retry=RETRY)
        print(f"  gripping at {pose[GRIPPER]}, load "
              f"{arms.read('Present_Load', GRIPPER, num_retry=RETRY)}")

        input("\n  CLEAR THE FLOOR, then press ENTER to drive...")
        head.enable_torque(list(WHEELS), num_retry=RETRY)

        period = 1.0 / args.rate
        t0 = time.monotonic()
        for leg in range(args.cycles):
            if stopping["flag"]:
                break
            sign = 1.0 if leg % 2 == 0 else -1.0
            leg_t0 = time.monotonic()
            total = ramp_t + args.dwell + ramp_t
            while not stopping["flag"]:
                lt = time.monotonic() - leg_t0
                if lt >= total:
                    break
                if lt < ramp_t:
                    v = args.accel * lt
                    phase = "ramp_up"
                elif lt < ramp_t + args.dwell:
                    v = args.peak_vel
                    phase = "dwell"
                else:
                    v = max(0.0, args.peak_vel - args.accel * (lt - ramp_t - args.dwell))
                    phase = "ramp_down"
                cmd = body_to_wheel_raw(sign * v, 0.0, 0.0)
                for n in WHEELS:
                    head.write("Goal_Velocity", n, cmd[n], num_retry=2)
                try:
                    actual = {n: head.read("Present_Velocity", n, normalize=False, num_retry=2)
                              for n in WHEELS}
                except Exception:
                    actual = dict.fromkeys(WHEELS, 0)
                ovx, ovy, oom = wheel_raw_to_body(actual)
                try:
                    av = arms.sync_read("Present_Position", list(WATCH), normalize=False)
                    al = arms.sync_read("Present_Load", list(WATCH), normalize=False)
                except Exception:
                    av = dict.fromkeys(WATCH, 0)
                    al = dict.fromkeys(WATCH, 0)
                rows.append(dict(
                    elapsed_s=round(time.monotonic() - t0, 3), leg=leg, sign=int(sign),
                    phase=phase, timestamp=datetime.now(timezone.utc).isoformat(),
                    cmd_vx=round(sign * v, 4), odom_vx=round(ovx, 4), odom_vy=round(ovy, 4),
                    odom_omega=round(oom, 2),
                    **{f"act_{n}": actual[n] for n in WHEELS},
                    **{f"pos_{n}": av.get(n, 0) for n in WATCH},
                    **{f"load_{n}": al.get(n, 0) for n in WATCH},
                    grip_dev=av.get(GRIPPER, grip_ref) - grip_ref))
                time.sleep(max(0.0, period - ((time.monotonic() - leg_t0) - lt)))
            stop_wheels(head, verify=False)
            print(f"    leg {leg+1}/{args.cycles} ({'fwd' if sign > 0 else 'rev'}) done"
                  f"  grip_dev {rows[-1]['grip_dev'] if rows else 0:+d}")
            end = time.monotonic() + args.pause
            while time.monotonic() < end and not stopping["flag"]:
                time.sleep(0.05)
    finally:
        ok = stop_wheels(head)
        print(f"\n  wheels stopped: {'VERIFIED' if ok else 'NOT VERIFIED -- CUT POWER'}")
        try:
            head.disable_torque(list(WHEELS), num_retry=RETRY)
        except Exception:
            pass
        print(f"  Lowering arms over {TORQUE_RELEASE_SECONDS:g}s...")
        try:
            release_gently(arms, arm_names)
        except Exception as exc:
            print(f"    [warn] gentle release failed: {type(exc).__name__}")
        for b in (arms, head):
            try:
                b.disable_torque(num_retry=RETRY)
            except Exception:
                pass
            try:
                b.disconnect(disable_torque=False)
            except Exception:
                pass

    if not rows:
        print("  no samples recorded.")
        return 1

    with (args.out / "base_payload.csv").open("w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    # Realized acceleration: delta-v over the ramp segment of each leg. Fitting the
    # ramp beats differentiating position, which is noise-dominated at 20 Hz.
    accs = []
    for leg in range(args.cycles):
        seg = [r for r in rows if r["leg"] == leg and r["phase"] == "ramp_up"]
        if len(seg) >= 4:
            dv = abs(seg[-1]["odom_vx"]) - abs(seg[0]["odom_vx"])
            dt = seg[-1]["elapsed_s"] - seg[0]["elapsed_s"]
            if dt > 0:
                accs.append(dv / dt)
    peak_dev = max(abs(r["grip_dev"]) for r in rows)
    dwell = [abs(r["odom_vx"]) for r in rows if r["phase"] == "dwell"]
    print("\n" + "=" * 60)
    print(f"  samples            {len(rows)} at {len(rows)/max(rows[-1]['elapsed_s'],1e-6):.1f} Hz")
    print(f"  commanded accel    {args.accel:g} m/s^2")
    if accs:
        print(f"  REALIZED accel     {sum(accs)/len(accs):.3f} m/s^2  "
              f"(ratio {sum(accs)/len(accs)/args.accel:.2f}, n={len(accs)} legs)")
    if dwell:
        print(f"  peak velocity      commanded {args.peak_vel:g}  realized {sum(dwell)/len(dwell):.3f} m/s")
    print(f"  max gripper drift  {peak_dev} counts (threshold {SLIP_COUNTS})")
    print(f"  VERDICT            {'SLIP' if peak_dev > SLIP_COUNTS else 'no slip detected'}")
    print(f"  data               {args.out / 'base_payload.csv'}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    sys.exit(main())
