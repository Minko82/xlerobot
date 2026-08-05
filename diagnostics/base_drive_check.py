#!/usr/bin/env python3
"""First-motion check for the omni base. Wheels only -- the arms are not touched.

Purpose is to answer three questions before the base is ever used in a real test:

  1. Do the wheels turn, and in the commanded direction?
  2. Does wheel-odometry read back the velocity that was commanded?
  3. Does the base stop reliably on every exit path?

Question 3 is the one that matters. A base that keeps driving after the script
dies is the only genuinely dangerous failure here, so the wheels are stopped in a
``finally``, on SIGINT, and again on SIGTERM -- and the stop is verified by reading
the speeds back rather than assumed.

Geometry (from XLerobotNewWiring): 3-wheel kiwi drive, wheels at 240/0/120
degrees, wheel radius 0.05 m, base radius 0.125 m.

Odometry substitutes for the IMU the protocol asks for -- this platform's D435
has none. Realized velocity is reconstructed from Present_Speed on all three
wheels, so it measures what the base actually did rather than what it was told.

Usage (start here -- 0.05 m/s for 2 s is a slow walk of a few centimetres):
    python diagnostics/base_drive_check.py --vx 0.05 --seconds 2

    # rotate in place instead
    python diagnostics/base_drive_check.py --omega 15 --seconds 2
"""

from __future__ import annotations

import argparse
import csv
import math
import signal
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus, OperatingMode

from xlerobot_pro.config import HEAD_PORT

RETRY = 5
WHEELS = ("base_left_wheel", "base_back_wheel", "base_right_wheel")

#: Geometry, matching XLerobotNewWiring._body_to_wheel_raw.
WHEEL_RADIUS = 0.05
BASE_RADIUS = 0.125
WHEEL_ANGLES_DEG = (240.0, 0.0, 120.0)

#: Hard ceiling on commanded wheel speed for a first-motion check. Well below the
#: robot's capability -- this script is not for going anywhere.
MAX_RAW_SPEED = 1500

#: Refuse anything faster than a slow walk. First motion is not the time to find
#: out the workspace was smaller than it looked.
MAX_VX = 0.15      # m/s
MAX_OMEGA = 45.0   # deg/s


def degps_to_raw(degps: float) -> int:
    raw = int(round(degps * 4096.0 / 360.0))
    return max(-0x8000, min(0x7FFF, raw))


def raw_to_degps(raw: int) -> float:
    return raw * 360.0 / 4096.0


def body_to_wheel_raw(vx: float, vy: float, omega_degps: float) -> dict[str, int]:
    """Body velocity (m/s, m/s, deg/s) -> per-wheel raw speed."""
    theta = math.radians(omega_degps)
    angles = [math.radians(a - 90.0) for a in WHEEL_ANGLES_DEG]
    out = {}
    degps = []
    for a in angles:
        linear = math.cos(a) * vx + math.sin(a) * vy + BASE_RADIUS * theta
        degps.append(linear / WHEEL_RADIUS * 180.0 / math.pi)
    raws = [abs(d) * 4096.0 / 360.0 for d in degps]
    peak = max(raws) if raws else 0.0
    if peak > MAX_RAW_SPEED:
        degps = [d * MAX_RAW_SPEED / peak for d in degps]
    for name, d in zip(WHEELS, degps):
        out[name] = degps_to_raw(d)
    return out


def wheel_raw_to_body(raw: dict[str, int]) -> tuple[float, float, float]:
    """Per-wheel raw speed -> body velocity. Least-squares inverse of the above."""
    angles = [math.radians(a - 90.0) for a in WHEEL_ANGLES_DEG]
    m = [[math.cos(a), math.sin(a), BASE_RADIUS] for a in angles]
    v = [raw_to_degps(raw[n]) * math.pi / 180.0 * WHEEL_RADIUS for n in WHEELS]
    # Solve the 3x3 system by Cramer's rule -- no numpy dependency needed.
    def det3(a):
        return (a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
                - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
                + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]))
    d = det3(m)
    if abs(d) < 1e-12:
        return 0.0, 0.0, 0.0
    sol = []
    for col in range(3):
        mm = [row[:] for row in m]
        for r in range(3):
            mm[r][col] = v[r]
        sol.append(det3(mm) / d)
    return sol[0], sol[1], math.degrees(sol[2])


def build_bus() -> FeetechMotorsBus:
    motors = {
        "head_motor_1": Motor(1, "sts3215", MotorNormMode.DEGREES),
        "head_motor_2": Motor(2, "sts3215", MotorNormMode.DEGREES),
        "base_left_wheel": Motor(3, "sts3215", MotorNormMode.RANGE_M100_100),
        "base_back_wheel": Motor(4, "sts3215", MotorNormMode.RANGE_M100_100),
        "base_right_wheel": Motor(5, "sts3215", MotorNormMode.RANGE_M100_100),
    }
    return FeetechMotorsBus(port=HEAD_PORT, motors=motors)


def stop_wheels(bus: FeetechMotorsBus, verify: bool = True) -> bool:
    """Command zero speed and confirm it took. Returns True if verified stopped."""
    for _ in range(3):
        for n in WHEELS:
            try:
                bus.write("Goal_Velocity", n, 0, num_retry=RETRY)
            except Exception:
                pass
        if not verify:
            return True
        time.sleep(0.15)
        try:
            speeds = [abs(bus.read("Present_Velocity", n, normalize=False, num_retry=RETRY))
                      for n in WHEELS]
            if max(speeds) <= 5:
                return True
        except Exception as exc:
            print(f"    [warn] could not verify stop: {type(exc).__name__}: {exc}")
            continue
    return False


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--vx", type=float, default=0.0, help="Forward velocity, m/s.")
    p.add_argument("--vy", type=float, default=0.0, help="Lateral velocity, m/s.")
    p.add_argument("--omega", type=float, default=0.0, help="Yaw rate, deg/s.")
    p.add_argument("--seconds", type=float, default=2.0, help="Drive duration.")
    p.add_argument("--ramp", type=float, default=0.5, help="Seconds to reach full speed.")
    p.add_argument("--rate", type=float, default=20.0, help="Command/log rate, Hz.")
    p.add_argument("--out", type=Path, default=None, help="Optional output directory.")
    args = p.parse_args()

    if abs(args.vx) > MAX_VX or abs(args.vy) > MAX_VX:
        print(f"FATAL: velocity capped at {MAX_VX} m/s for a first-motion check.")
        return 1
    if abs(args.omega) > MAX_OMEGA:
        print(f"FATAL: yaw rate capped at {MAX_OMEGA} deg/s for a first-motion check.")
        return 1
    if args.seconds > 10:
        print("FATAL: duration capped at 10 s for a first-motion check.")
        return 1
    if not any((args.vx, args.vy, args.omega)):
        print("Nothing commanded. Give --vx, --vy or --omega.")
        return 1

    target = body_to_wheel_raw(args.vx, args.vy, args.omega)
    print("\n  BASE DRIVE CHECK")
    print(f"    commanded   vx={args.vx} m/s  vy={args.vy} m/s  omega={args.omega} deg/s")
    print(f"    duration    {args.seconds:g} s   ramp {args.ramp:g} s")
    print("    wheel raw   " + "  ".join(f"{n.replace('base_','').replace('_wheel',''):>5}={v}"
                                         for n, v in target.items()))
    dist = abs(args.vx) * args.seconds
    print(f"    approx travel {dist*100:.0f} cm")
    print("\n  THE BASE WILL MOVE. Clear the floor, and check nothing is tethered")
    print("  that could be dragged -- power leads especially.")
    input("  Press ENTER when clear (Ctrl-C at any time stops the wheels)...")

    bus = build_bus()
    bus.connect()

    stopping = {"flag": False}

    def _stop(*_):
        stopping["flag"] = True

    signal.signal(signal.SIGINT, _stop)
    signal.signal(signal.SIGTERM, _stop)

    rows = []
    read_failed = {"flag": False}
    try:
        for n in WHEELS:
            bus.write("Operating_Mode", n, OperatingMode.VELOCITY.value, num_retry=RETRY)
        bus.enable_torque(list(WHEELS), num_retry=RETRY)

        t0 = time.monotonic()
        period = 1.0 / args.rate
        while not stopping["flag"]:
            t = time.monotonic() - t0
            if t >= args.seconds:
                break
            # Linear ramp in and out, so the base never steps to full speed.
            scale = min(1.0, t / args.ramp) if args.ramp > 0 else 1.0
            remain = args.seconds - t
            if args.ramp > 0 and remain < args.ramp:
                scale = min(scale, remain / args.ramp)
            for n in WHEELS:
                bus.write("Goal_Velocity", n, int(round(target[n] * scale)), num_retry=RETRY)
            try:
                actual = {n: bus.read("Present_Velocity", n, normalize=False, num_retry=2)
                          for n in WHEELS}
            except Exception as exc:
                # Do NOT silently zero this. A telemetry failure that reads as
                # "not moving" is indistinguishable from a base that will not move,
                # and that ambiguity already cost one run.
                if not read_failed["flag"]:
                    print(f"    [ERROR] Present_Velocity read failed: {type(exc).__name__}: {exc}")
                    read_failed["flag"] = True
                actual = dict.fromkeys(WHEELS, 0)
            vx, vy, om = wheel_raw_to_body(actual)
            rows.append(dict(elapsed_s=round(t, 3),
                             timestamp=datetime.now(timezone.utc).isoformat(),
                             scale=round(scale, 3),
                             **{f"cmd_{n}": int(round(target[n] * scale)) for n in WHEELS},
                             **{f"act_{n}": actual[n] for n in WHEELS},
                             odom_vx=round(vx, 4), odom_vy=round(vy, 4), odom_omega=round(om, 2)))
            time.sleep(max(0.0, period - ((time.monotonic() - t0) - t)))
    finally:
        ok = stop_wheels(bus)
        print(f"\n  wheels stopped: {'VERIFIED' if ok else 'NOT VERIFIED -- CUT POWER'}")
        try:
            bus.disable_torque(list(WHEELS), num_retry=RETRY)
        except Exception:
            pass
        try:
            bus.disconnect(disable_torque=False)
        except Exception:
            pass

    if rows:
        moving = [r for r in rows if r["scale"] > 0.9]
        if moving:
            mvx = sum(r["odom_vx"] for r in moving) / len(moving)
            mvy = sum(r["odom_vy"] for r in moving) / len(moving)
            mom = sum(r["odom_omega"] for r in moving) / len(moving)
            print("\n" + "=" * 58)
            print(f"  samples            {len(rows)} at {len(rows)/max(rows[-1]['elapsed_s'],1e-6):.1f} Hz")
            print(f"  commanded          vx {args.vx:+.3f}  vy {args.vy:+.3f}  omega {args.omega:+.1f}")
            print(f"  odometry (at speed) vx {mvx:+.3f}  vy {mvy:+.3f}  omega {mom:+.1f}")
            if abs(args.vx) > 1e-6:
                print(f"  vx ratio           {mvx/args.vx:.2f}  (1.00 = odometry agrees)")
            if abs(args.omega) > 1e-6:
                print(f"  omega ratio        {mom/args.omega:.2f}")
            print("=" * 58)
        if args.out:
            args.out.mkdir(parents=True, exist_ok=True)
            with (args.out / "base_drive.csv").open("w", newline="") as fh:
                w = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
                w.writeheader()
                w.writerows(rows)
            print(f"  data {args.out / 'base_drive.csv'}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
