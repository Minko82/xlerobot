#!/usr/bin/env python3
"""Log Jetson temperature, power, and clocks for the thermal endurance tests.

Runs on the Jetson. Wraps ``tegrastats`` and writes one CSV row per sample,
plus a second CSV of manual annotations (battery state-of-charge, thermocouple
readings) that you type in while the run is in progress.

Usage (one command per test phase):

    python diagnostics/log_thermal_power.py --phase active-idle       --minutes 60
    python diagnostics/log_thermal_power.py --phase peak-inference    --minutes 60
    python diagnostics/log_thermal_power.py --phase actuator-load     --minutes 60

While it runs you can type annotations and press ENTER:

    bat 87        battery state of charge, percent
    seat 41.2     servo-housing / PLA-seat temperature, degrees C
    note ...      free text

Output goes to results/thermal/<phase>_<timestamp>/ as samples.csv,
annotations.csv, and tegrastats_raw.log. Stop early with Ctrl-C; the CSVs are
flushed on every sample, so nothing is lost.
"""

import argparse
import csv
import glob
import os
import re
import subprocess
import sys
import threading
import time
from datetime import datetime, timezone
from pathlib import Path

# ---------------------------------------------------------------------------
# tegrastats parsing
#
# The field layout differs between JetPack releases and between Xavier and
# Orin, so every pattern is optional and missing values are recorded as blank
# rather than guessed. The raw line is always kept in tegrastats_raw.log.
# ---------------------------------------------------------------------------

PATTERNS = {
    # Orin uses lowercase gpu@/cpu@/tj@; older boards use GPU@/BCPU@.
    "gpu_temp_c": r"(?:^|\s)(?:gpu|GPU)@(-?[\d.]+)C",
    "cpu_temp_c": r"(?:^|\s)(?:cpu|BCPU|MCPU)@(-?[\d.]+)C",
    "tj_temp_c": r"(?:^|\s)tj@(-?[\d.]+)C",
    "soc0_temp_c": r"(?:^|\s)soc0@(-?[\d.]+)C",
    # GPU utilisation. Some builds append the clock: "GR3D_FREQ 45%@1300".
    "gpu_util_pct": r"GR3D_FREQ\s+(\d+)%",
    "gpu_clock_mhz": r"GR3D_FREQ\s+\d+%@(\d+)",
    "emc_util_pct": r"EMC_FREQ\s+(\d+)%",
    # Power rails: "VDD_IN 4517mW/4517mW" or "VDD_IN 1908/1908".
    "vdd_in_mw": r"VDD_IN\s+(\d+)(?:mW)?/",
    "vdd_cpu_gpu_cv_mw": r"VDD_CPU_GPU_CV\s+(\d+)(?:mW)?/",
    "vdd_soc_mw": r"VDD_SOC\s+(\d+)(?:mW)?/",
    "ram_used_mb": r"RAM\s+(\d+)/\d+MB",
}

CPU_BLOCK = re.compile(r"CPU\s+\[([^\]]+)\]")
CPU_CORE = re.compile(r"(\d+)%@(\d+)")

# tegrastats only appends the clock to GR3D_FREQ on some builds
# ("GR3D_FREQ 45%@1300"). JetPack 6.2 on Orin emits a bare "GR3D_FREQ 23%", so
# the regex above yields nothing and throttling becomes undetectable. devfreq
# sysfs always carries the frequency, so it is the reliable source.
_GPU_DEVFREQ = next((p for p in sorted(glob.glob("/sys/class/devfreq/*.gpu")) if os.path.isdir(p)), None)


def read_gpu_clock_sysfs() -> tuple[str, str]:
    """Return (current_mhz, max_mhz) from devfreq, or blanks if unavailable.

    max_mhz is the throttle reference: if the *achievable* peak falls below it
    while load stays high, the GPU is being clamped.
    """
    if not _GPU_DEVFREQ:
        return "", ""
    try:
        with open(f"{_GPU_DEVFREQ}/cur_freq") as handle:
            current = int(handle.read().strip()) // 1_000_000
        with open(f"{_GPU_DEVFREQ}/max_freq") as handle:
            maximum = int(handle.read().strip()) // 1_000_000
    except (OSError, ValueError):
        return "", ""
    return str(current), str(maximum)

FIELDS = [
    "elapsed_s",
    "timestamp",
    "gpu_temp_c",
    "cpu_temp_c",
    "tj_temp_c",
    "soc0_temp_c",
    "gpu_util_pct",
    "gpu_clock_mhz",
    "gpu_max_mhz",
    "emc_util_pct",
    "vdd_in_mw",
    "vdd_cpu_gpu_cv_mw",
    "vdd_soc_mw",
    "ram_used_mb",
    "cpu_util_mean_pct",
    "cpu_clock_max_mhz",
]


def parse_tegrastats(line: str) -> dict:
    """Pull the fields we care about out of one tegrastats line."""
    row = {}
    for name, pattern in PATTERNS.items():
        match = re.search(pattern, line)
        row[name] = match.group(1) if match else ""

    # Aggregate the per-core CPU block into a mean load and a peak clock.
    # A peak clock that sags while load stays high is the signature of
    # thermal throttling.
    cpu_block = CPU_BLOCK.search(line)
    if cpu_block:
        cores = CPU_CORE.findall(cpu_block.group(1))
        if cores:
            loads = [int(load) for load, _ in cores]
            clocks = [int(clock) for _, clock in cores]
            row["cpu_util_mean_pct"] = f"{sum(loads) / len(loads):.1f}"
            row["cpu_clock_max_mhz"] = str(max(clocks))
    row.setdefault("cpu_util_mean_pct", "")
    row.setdefault("cpu_clock_max_mhz", "")

    # Fall back to devfreq when tegrastats omitted the clock, and always record
    # the ceiling so throttling can be judged against it after the fact.
    current_mhz, max_mhz = read_gpu_clock_sysfs()
    if not row["gpu_clock_mhz"]:
        row["gpu_clock_mhz"] = current_mhz
    row["gpu_max_mhz"] = max_mhz
    return row


def read_annotations(out_dir: Path, stop: threading.Event) -> None:
    """Append typed annotations to annotations.csv until the run ends."""
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
            if kind not in {"bat", "seat", "note"}:
                kind, value = "note", entry
            writer.writerow([datetime.now(timezone.utc).isoformat(), kind, value])
            handle.flush()
            print(f"  [logged] {kind} {value}", flush=True)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--phase",
        required=True,
        choices=["active-idle", "peak-inference", "actuator-load"],
        help="Which test phase this run corresponds to.",
    )
    parser.add_argument(
        "--minutes", type=float, default=60.0, help="Run duration in minutes (default 60)."
    )
    parser.add_argument(
        "--interval-ms", type=int, default=1000, help="tegrastats sample interval (default 1000)."
    )
    parser.add_argument(
        "--out-root",
        type=Path,
        default=Path("results/thermal"),
        help="Directory to write results into (default results/thermal).",
    )
    args = parser.parse_args()

    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    out_dir = args.out_root / f"{args.phase}_{stamp}"
    out_dir.mkdir(parents=True, exist_ok=True)

    try:
        proc = subprocess.Popen(
            ["tegrastats", "--interval", str(args.interval_ms)],
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            bufsize=1,
        )
    except FileNotFoundError:
        print(
            "tegrastats not found. This script must run on the Jetson, not on a laptop.",
            file=sys.stderr,
        )
        return 1

    deadline = time.monotonic() + args.minutes * 60.0
    started = time.monotonic()
    stop = threading.Event()

    annotator = threading.Thread(target=read_annotations, args=(out_dir, stop), daemon=True)
    annotator.start()

    print(f"Logging '{args.phase}' for {args.minutes:g} min into {out_dir}")
    print("Type 'bat 87', 'seat 41.2', or 'note ...' then ENTER to annotate. Ctrl-C to stop.\n")

    peak = {"gpu_temp_c": None, "vdd_in_mw": None}
    samples = 0

    try:
        with (
            (out_dir / "samples.csv").open("w", newline="") as csv_handle,
            (out_dir / "tegrastats_raw.log").open("w") as raw_handle,
        ):
            writer = csv.DictWriter(csv_handle, fieldnames=FIELDS)
            writer.writeheader()

            for line in proc.stdout:
                now = time.monotonic()
                if now >= deadline:
                    break

                raw_handle.write(line)
                raw_handle.flush()

                row = parse_tegrastats(line)
                row["elapsed_s"] = f"{now - started:.1f}"
                row["timestamp"] = datetime.now(timezone.utc).isoformat()
                writer.writerow(row)
                csv_handle.flush()
                samples += 1

                # Track running maxima so the summary matches the values the
                # protocol asks you to write into the results table.
                for key in peak:
                    if row.get(key):
                        value = float(row[key])
                        if peak[key] is None or value > peak[key]:
                            peak[key] = value

                # Heartbeat once a minute so you can see it is still alive.
                if samples % max(1, int(60_000 / args.interval_ms)) == 0:
                    elapsed_min = (now - started) / 60.0
                    gpu = row.get("gpu_temp_c") or "?"
                    watts = (
                        f"{float(row['vdd_in_mw']) / 1000:.1f}" if row.get("vdd_in_mw") else "?"
                    )
                    print(
                        f"  {elapsed_min:5.1f} min | GPU {gpu} C | {watts} W | "
                        f"peak GPU {peak['gpu_temp_c']} C",
                        flush=True,
                    )
    except KeyboardInterrupt:
        print("\nStopped early by Ctrl-C.")
    finally:
        stop.set()
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except subprocess.TimeoutExpired:
            proc.kill()

    peak_watts = f"{peak['vdd_in_mw'] / 1000:.1f} W" if peak["vdd_in_mw"] else "n/a"
    print("\n" + "=" * 60)
    print(f"phase           {args.phase}")
    print(f"samples         {samples}")
    print(f"max GPU temp    {peak['gpu_temp_c']} C   <- A2 results table")
    print(f"peak VDD_IN     {peak_watts}")
    print(f"data            {out_dir}")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
