#!/usr/bin/env python3
"""A1 — boot-up current sequencing and compute-rail brownout margin.

Enables every actuator one at a time in a fixed order, then commands a
simultaneous high-torque pose at full count, while sampling the compute rail as
fast as the kernel allows. Repeats for n trials.

Compute-rail voltage comes from the Jetson's onboard INA3221 rather than an
oscilloscope::

    /sys/class/hwmon/hwmon1/in1_input     VDD_IN, millivolts
    /sys/class/hwmon/hwmon1/curr1_input   VDD_IN, milliamps

Measured throughput is ~760 Hz with ~8 mV resolution. That is below the
protocol's 1 kHz and the INA3221 averages internally, so a very short transient
may be smoothed: **the reported V_min is an upper bound on the true dip.** Say so
when reporting. It is entirely adequate for inrush lasting tens of milliseconds,
which is the regime that actually resets the compute module.

Motor-bus current is NOT measured here -- curr1_input is the Jetson's own draw.
That column comes from the bench supply.

Every sample is flushed to disk as it is taken, so a brownout that resets the
board loses at most the final sample rather than the whole trial. A reset is
detected by comparing the kernel boot id across the trial.

Usage:
    python diagnostics/a1_brownout.py --config baseline --trials 10 --out results/A1/baseline
    python diagnostics/a1_brownout.py --config two-bus  --trials 10 --out results/A1/two_bus
"""

from __future__ import annotations

import argparse
import csv
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

from lerobot.motors import Motor, MotorNormMode
from lerobot.motors.feetech import FeetechMotorsBus

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))
from xlerobot_pro.firmware_limits import (  # noqa: E402
    ARM_ACCELERATION,
    ARM_TORQUE_LIMIT,
    TORQUE_RELEASE_SECONDS,
)

ARMS_PORT = "/dev/xle_arms"
HEAD_PORT = "/dev/xle_head"
JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll", "gripper"]

RAIL_V = Path("/sys/class/hwmon/hwmon1/in1_input")
RAIL_I = Path("/sys/class/hwmon/hwmon1/curr1_input")
BOOT_ID = Path("/proc/sys/kernel/random/boot_id")

RETRY = 5

#: Seconds to hold at each step of the enable sequence.
STEP_SECONDS = 2.0

#: Active-motor counts the protocol tabulates.
REPORT_COUNTS = (1, 4, 8, 12, 17)


def build_buses():
    """Bus 1 = both arms (1-12). Bus 2 = head (1-2) + wheels (3-5)."""
    arms = {f"left_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES) for j, i in zip(JOINTS, range(1, 7))}
    arms |= {f"right_{j}": Motor(i, "sts3215", MotorNormMode.DEGREES) for j, i in zip(JOINTS, range(7, 13))}
    head = {"head_motor_1": Motor(1, "sts3215", MotorNormMode.DEGREES),
            "head_motor_2": Motor(2, "sts3215", MotorNormMode.DEGREES),
            "base_left_wheel": Motor(3, "sts3215", MotorNormMode.DEGREES),
            "base_back_wheel": Motor(4, "sts3215", MotorNormMode.DEGREES),
            "base_right_wheel": Motor(5, "sts3215", MotorNormMode.DEGREES)}
    return (("arms", FeetechMotorsBus(port=ARMS_PORT, motors=arms)),
            ("head", FeetechMotorsBus(port=HEAD_PORT, motors=head)))


def enable_order() -> list[tuple[str, str]]:
    """(bus, motor) in the protocol's order: neck, left arm, right arm, wheels."""
    order = [("head", "head_motor_1"), ("head", "head_motor_2")]
    order += [("arms", f"left_{j}") for j in JOINTS]
    order += [("arms", f"right_{j}") for j in JOINTS]
    order += [("head", n) for n in ("base_left_wheel", "base_back_wheel", "base_right_wheel")]
    return order


def boot_id() -> str:
    try:
        return BOOT_ID.read_text().strip()
    except OSError:
        return "unknown"


class RailSampler(threading.Thread):
    """Polls the INA3221 as fast as the kernel allows, flushing every sample.

    Flushing per sample is deliberate: the whole point of this test is to provoke
    a brownout, and buffered rows would be lost with the process.
    """

    def __init__(self, path: Path, stop: threading.Event):
        super().__init__(daemon=True)
        self.stop = stop
        self.fh = open(path, "w", newline="")
        self.w = csv.writer(self.fh)
        self.w.writerow(["elapsed_s", "timestamp", "millivolts", "milliamps",
                         "active_motors", "phase"])
        self.active = 0
        self.phase = "idle"
        self.n = 0
        self.t0 = time.monotonic()

    def run(self):
        while not self.stop.is_set():
            try:
                mv = int(RAIL_V.read_text())
                ma = int(RAIL_I.read_text())
            except (OSError, ValueError):
                continue
            self.w.writerow([f"{time.monotonic() - self.t0:.4f}",
                             datetime.now(timezone.utc).isoformat(),
                             mv, ma, self.active, self.phase])
            self.fh.flush()
            self.n += 1
        self.fh.close()


def release_gently(buses, names, seconds=TORQUE_RELEASE_SECONDS, steps=24):
    """Bleed torque away rather than cutting it, so nothing drops."""
    try:
        for i in range(steps - 1, -1, -1):
            limit = int(ARM_TORQUE_LIMIT * i / steps)
            for label, bus in buses:
                for name in names.get(label, []):
                    try:
                        bus.write("Torque_Limit", name, limit, num_retry=RETRY)
                    except Exception:
                        pass
            time.sleep(seconds / steps)
    finally:
        for label, bus in buses:
            for name in names.get(label, []):
                try:
                    bus.write("Torque_Limit", name, ARM_TORQUE_LIMIT, num_retry=RETRY)
                except Exception:
                    pass


def run_trial(trial: int, out_dir: Path, args) -> dict:
    """One enable sequence plus a worst-case simultaneous pose command."""
    stop = threading.Event()
    sampler = RailSampler(out_dir / f"trial_{trial:02d}_rail.csv", stop)

    buses = build_buses()
    for _, bus in buses:
        bus.connect()
    by_bus = {"arms": [], "head": []}
    boot_before = boot_id()

    sampler.start()
    time.sleep(1.0)  # a second of quiescent baseline before anything is enabled

    try:
        for idx, (label, motor) in enumerate(enable_order(), start=1):
            bus = dict(buses)[label]
            sampler.phase = f"enable_{motor}"
            bus.write("Torque_Limit", motor, ARM_TORQUE_LIMIT, num_retry=RETRY)
            bus.write("Acceleration", motor, ARM_ACCELERATION, num_retry=RETRY)
            bus.enable_torque([motor], num_retry=RETRY)
            by_bus[label].append(motor)
            sampler.active = idx
            time.sleep(args.step_seconds)

        # Worst-case: command every joint to the reference pose at once.
        sampler.phase = "simultaneous_pose"
        if args.pose_file.exists():
            import json
            targets = {k: {n: int(v) for n, v in d.items()}
                       for k, d in json.loads(args.pose_file.read_text()).items()}
            for label, bus in buses:
                sel = {n: v for n, v in targets.get(label, {}).items() if n in by_bus[label]}
                if sel:
                    bus.sync_write("Goal_Position", sel, normalize=False, num_retry=RETRY)
        time.sleep(3.0)
        sampler.phase = "settled"
        time.sleep(2.0)
    finally:
        sampler.phase = "release"
        release_gently(buses, by_bus)
        for label, bus in buses:
            try:
                if by_bus[label]:
                    bus.disable_torque(by_bus[label], num_retry=RETRY)
            except Exception:
                pass
            try:
                bus.disconnect()
            except Exception:
                pass
        stop.set()
        sampler.join(timeout=5)

    boot_after = boot_id()
    rows = list(csv.DictReader(open(out_dir / f"trial_{trial:02d}_rail.csv")))
    mv = [int(r["millivolts"]) for r in rows] or [0]
    ma = [int(r["milliamps"]) for r in rows] or [0]

    per_count = {}
    for c in REPORT_COUNTS:
        sel = [int(r["millivolts"]) for r in rows if int(r["active_motors"]) == c]
        cur = [int(r["milliamps"]) for r in rows if int(r["active_motors"]) == c]
        if sel:
            per_count[c] = (min(sel), max(cur))

    return dict(trial=trial, config=args.config, samples=len(rows),
                sample_hz=round(len(rows) / max(float(rows[-1]["elapsed_s"]), 1e-6), 1) if rows else 0,
                v_min_mv=min(mv), v_idle_mv=mv[0] if mv else 0, i_peak_ma=max(ma),
                reset="Y" if boot_before != boot_after else "N",
                per_count=per_count)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--config", required=True, choices=["baseline", "two-bus"],
                   help="Wiring configuration in use. Recorded, not detected -- set it honestly.")
    p.add_argument("--trials", type=int, default=10, help="Protocol asks for n >= 10.")
    p.add_argument("--out", type=Path, required=True)
    p.add_argument("--step-seconds", type=float, default=STEP_SECONDS)
    p.add_argument("--pose-file", type=Path, default=Path("calibration/hold_pose.json"))
    args = p.parse_args()

    if not RAIL_V.exists():
        print(f"FATAL: {RAIL_V} missing -- INA3221 rail monitoring unavailable.")
        return 1

    args.out.mkdir(parents=True, exist_ok=True)
    print(f"\n  A1 brownout margin -- config={args.config}, {args.trials} trials")
    print(f"  Rail: {RAIL_V} (compute rail only; bus current comes from the bench supply)")
    print("  ARMS WILL MOVE at full motor count. Keep clear.")
    input("  Press ENTER to begin...")

    summary = []
    for t in range(1, args.trials + 1):
        print(f"\n  --- trial {t}/{args.trials} ---")
        r = run_trial(t, args.out, args)
        summary.append(r)
        print(f"    {r['samples']} samples @ {r['sample_hz']:.0f} Hz | "
              f"idle {r['v_idle_mv']} mV | V_min {r['v_min_mv']} mV | "
              f"I_peak {r['i_peak_ma']} mA | reset {r['reset']}")
        if t < args.trials:
            time.sleep(3.0)

    with open(args.out / "trials.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["trial", "config", "samples", "sample_hz", "v_idle_mv",
                    "v_min_mv", "i_peak_ma", "reset"])
        for r in summary:
            w.writerow([r["trial"], r["config"], r["samples"], r["sample_hz"],
                        r["v_idle_mv"], r["v_min_mv"], r["i_peak_ma"], r["reset"]])

    with open(args.out / "by_motor_count.csv", "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(["active_motors", "config", "v_min_mv", "i_peak_ma", "trials"])
        for c in REPORT_COUNTS:
            vs = [r["per_count"][c][0] for r in summary if c in r["per_count"]]
            is_ = [r["per_count"][c][1] for r in summary if c in r["per_count"]]
            if vs:
                w.writerow([c, args.config, min(vs), max(is_), len(vs)])

    resets = sum(1 for r in summary if r["reset"] == "Y")
    vmin = min(r["v_min_mv"] for r in summary)
    print("\n" + "=" * 58)
    print(f"  config      {args.config}")
    print(f"  trials      {len(summary)}")
    print(f"  V_min       {vmin} mV  ({vmin/1000:.3f} V)")
    print(f"  resets      {resets}")
    print(f"  data        {args.out}")
    print("  NOTE: V_min is an upper bound -- the INA3221 averages internally and")
    print("        sampling is ~760 Hz, so sub-millisecond dips may be smoothed.")
    print("=" * 58)
    return 0


if __name__ == "__main__":
    sys.exit(main())
