# Test C2 — Inference Optimization & Control-Rate Characterization

**Status:** protocol drafted 2026-08-01. Stage 1 has preliminary n=1 data (below);
Stages 2 and 3 not started.

## PURPOSE

Establish the achievable control rate for onboard VLA inference on the Jetson,
identify which parameters actually govern it, and determine how far inference
latency can be reduced before task success degrades.

Motivated by a finding during A2: the platform was running at **5.01 Hz control
frequency** purely because of a benchmark script default (`ACTION_CHUNK = 5`),
while the model natively predicts 50 actions per forward pass. Correcting that
alone yields **55.4 Hz at identical compute cost**. This test exists to
characterize the real envelope rather than an accidental operating point.

## EQUIPMENT

- Jetson Orin Nano Super 8 GB, JetPack 6.2.1, MAXN_SUPER
- SmolVLA checkpoint (`rlodhi/smolvla_screw_picking_5000`, fp16)
- `~/vla/smolvla_benchmark_jetson.py` — latency harness
- `diagnostics/log_thermal_power.py` — 1 Hz thermal/power/clock logging
- Standard object set + fixed grasp fixture (Stage 3 only)
- Ambient thermometer

## DEFINITIONS

| Term | Meaning |
|---|---|
| **Replan rate** | 1 / end-to-end latency. How often the policy sees new observations. |
| **Control rate** | replan_rate x n_action_steps. How fast actions reach the motors. |
| **Open-loop horizon** | n_action_steps / control_rate. Seconds committed between observations. |

These are distinct and must not be conflated. Reporting only one is misleading:
a high control rate with a low replan rate means smooth motion but slow reaction.

---

## STAGE 1 — Latency characterization

Sweep each factor, **n >= 3 per configuration**, 60 s per run, DVFS unless stated.

| Factor | Levels |
|---|---|
| Denoising steps (`--num-steps`) | 10, 5, 3, 2 |
| Action chunk (`--chunk`) | 5, 25, 50 |
| Clock policy | DVFS, `jetson_clocks` |
| Graph capture | off, `torch.compile(mode="reduce-overhead")` |

Record median forward latency, replan Hz, control Hz, GPU clock, GPU util.
Fit `T = fixed + steps x per_step` and report both terms — the fixed term is the
floor no parameter tuning can cross.

### RESULTS (RECORD HERE)

| Config | n | Latency (ms) | Replan (Hz) | Control (Hz) | GPU clock (MHz) |
|---|---|---|---|---|---|
| 10 steps, chunk 5, DVFS | | | | | |
| 10 steps, chunk 50, DVFS | | | | | |
| 5 steps, chunk 50, DVFS | | | | | |
| 3 steps, chunk 50, DVFS | | | | | |
| 2 steps, chunk 50, DVFS | | | | | |
| 10 steps, chunk 50, jetson_clocks | | | | | |
| 10 steps, chunk 50, torch.compile | | | | | |

Summary -> per_step = ____ ms; fixed floor = ____ ms

---

## STAGE 2 — Thermal cost of the selected configuration

30 min continuous inference at the chosen config, logging GPU temperature and
clock at 1 Hz. Compare peak against the A2 baseline.

Any clock-policy change must report its thermal penalty alongside its latency
gain. `jetson_clocks` prevents the GPU idling at 306 MHz, so the board runs
warmer at rest and the fan sits higher.

### RESULTS (RECORD HERE)

| Config | Duration | Max GPU (C) | Rise above ambient | Mean power (W) | Throttle |
|---|---|---|---|---|---|
| A2 baseline (10 steps, chunk 5, DVFS) | 30 min | 59.9 | +35.5 | 9.40 | N |
| Selected config | | | | | |

---

## STAGE 3 — Task-success validation

**This stage is what makes Stage 1 publishable.** Latency reductions from fewer
denoising steps trade against action quality, and Stage 1 measures only speed.

For each candidate config, **n >= 20 grasp attempts** on the standard object set
from a fixed fixture. Record success/failure per attempt.

A latency reduction is reportable **only if** success rate is statistically
indistinguishable from the 10-step baseline (two-proportion test, alpha = 0.05).
Report the confidence interval, not just the point estimate.

### RESULTS (RECORD HERE)

| Config | Attempts | Successes | Rate (%) | 95% CI | vs baseline (p) |
|---|---|---|---|---|---|
| 10 steps (baseline) | | | | | — |
| 5 steps | | | | | |
| 3 steps | | | | | |
| 2 steps | | | | | |

Summary -> minimum denoising steps preserving baseline success = ____

---

## PRELIMINARY DATA (n = 1, 2026-08-01)

Collected during A2 as an informal sweep. **Not sufficient for publication** —
n=1, and no task-success validation. Retained to inform Stage 1 design.

Conditions: JetPack 6.2.1, MAXN_SUPER, ambient 24.4 C, all motors torque-free.

### Denoising steps (DVFS, chunk 5)

| Steps | Latency (ms) | Replan (Hz) |
|---|---|---|
| 10 | 994.27 | 1.002 |
| 5 | 605.87 | 1.644 |
| 3 | 443.73 | 2.243 |
| 2 | 352.93 | 2.805 |

Fitted: **~80 ms per denoising step, ~195 ms fixed floor.**
Predictions from the first two points matched measurement within 1.6% (3 steps)
and 5% (2 steps), so the linear model is sound over this range.

### Action chunk (jetson_clocks, 10 steps)

| Chunk | Latency (ms) | Control (Hz) |
|---|---|---|
| 5 | 917.72 | 5.44 |
| 25 | 909.60 | 27.45 |
| 50 | 900.82 | 55.42 |

**Latency is flat across a 10x range in chunk size.** The action expert computes
all 50 predictions regardless; chunk only selects how many are consumed before
replanning. Chunk is therefore a free parameter for control rate, costing only
open-loop horizon.

### Clock policy (10 steps, chunk 5)

| Policy | Latency (ms) | Delta |
|---|---|---|
| DVFS | 994.27 | — |
| jetson_clocks | 909.56 | **-8.5%** |

Pinning the GPU from a floating 306-429 MHz to a locked 1020 MHz — more than
doubling the average clock — recovered only 85 ms. Combined with 23-30% GPU
utilization and one CPU core at 90%, this indicates the ~195 ms floor is
dominated by **CPU-side dispatch and kernel-launch overhead**, not GPU compute.
`torch.compile` with CUDA graphs is therefore the only remaining lever on the
fixed term, and Stage 1 should test it.

---

## KNOWN CONSTRAINTS

- **`--chunk` must not exceed the checkpoint's native `chunk_size` (50).**
  Values below it discard predictions; values above it are undefined.
- **Model loading requires a CPU-side fp16 cast.** The checkpoint config pins
  `device="cuda"`, which makes `from_pretrained` load full-precision weights
  straight to GPU and OOM an 8 GB Jetson. `smolvla_benchmark_jetson.py`
  overrides the config to CPU, casts, then moves. See its `load_policy()`.
- **`expandable_segments:True` is unusable on Tegra** — asserts on NVML.
- **`jetson_clocks --store` before applying**, or `--restore` fails with
  "conf file not found" and only a reboot returns the board to DVFS.
