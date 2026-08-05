# XLeRobot-Pro — Protocol Results Tables

Filled from measured data as of 2026-08-04. Ambient **24.4 °C (76 °F)** throughout.
Blank cells are genuinely unmeasured — nothing here is estimated or carried over
from another condition.

Platform: Jetson Orin Nano Super 8 GB, JetPack 6.2.1, MAXN_SUPER, DVFS unless stated.

> **Wiring note for the writeup:** the protocol says "Tri-Bus" throughout. This
> robot is **2-bus** (arms bus = 12 motors; head/base bus = 5 motors). Any A1
> comparison must be described as 2-bus, not tri-bus.

---

## A1 — Boot-up current sequencing & brownout margin

**NOT RUN — equipment-blocked.**

| Active motors | Config | Bus current (A) | Compute-rail V_min (V) | Reset? |
|---|---|---|---|---|
| 1 / 4 / 8 / 12 / 17 | Baseline | — | — | — |
| 1 / 4 / 8 / 12 / 17 | 2-Bus | — | — | — |

Available: bench DC supply with current logging.
Missing: **oscilloscope (≥1 kHz) for compute-rail V_min** — the headline number.
Also required for the baseline arm: rewiring the original daisy-chained harness.

`Reset? (Y/N)` is recoverable without a scope (a brownout reset shows as a dropped
SSH session plus reset `uptime`), so a partial A1 could fill 2 of 3 columns.

---

## A2 — Sustained thermal endurance & untethered runtime

| Phase | Duration | Max GPU (°C) | Rise vs Ambient (°C) | Draw (%/h) | Throttle |
|---|---|---|---|---|---|
| Active-idle 1 | 60 min | 51.0 | +26.6 | 4.19 | N |
| Active-idle 2 | 60 min | 51.2 | +26.8 | 4.88 | N |
| Active-idle 3 | 60 min | 51.1 | +26.7 | 5.16 | N |
| **Active-idle mean** | | **51.1** | **+26.7** | **4.74** | **N** |
| Peak inference 1 | 30 min | 59.9 | +35.5 | 7.95 | N |
| Peak inference 2 | 30 min | 59.9 | +35.5 | *distorted* | N |
| Peak inference 3 | 30 min | 60.1 | +35.7 | 6.36 | N |
| **Peak inference mean** | | **59.97** | **+35.57** | **6.4–8.0** | **N** |
| Sustained actuator load 1 | 30 min | 51.4 | +27.0 | 4.68 | N |
| Sustained actuator load 2 | 30 min | 51.7 | +27.3 | 4.59 | N |
| Sustained actuator load 3 | 30 min | 51.5 | +27.1 | 6.19 | N |
| **Sustained actuator load mean** | | **51.54** | **+27.14** | **5.15** | **N** |
| **Peak inference + actuator load** | 30 min | **61.1** | **+36.7** | **8.15** | **N** |

**Summary → max GPU = 61.1 °C** (combined inference + actuator load; 60.1 °C for inference alone). Standard config: 10 denoising steps, DVFS.
**Projected runtime**: idle **21.1 h**; under continuous inference **12.5–15.6 h**.

### Caveats that belong in the writeup

- **Peak-inference reproducibility is exceptional** — 0.2 °C spread across three
  runs, idle power within 0.01 W.
- **Battery drain near full charge is unusable.** The C300 held 100% for 935 s of
  continuous ~10 W draw, then shed 3% in 370 s. Only mid-range (≈45–90%) windows
  are quoted. Recommend measuring current at the 12 V rail instead.
- **Throttle = N is valid at 24.4 °C only.** Rise above ambient is +35.6 °C; the
  same workload at 40 °C ambient would reach ~76 °C, near throttle onset. Do not
  generalise.
- **Actuator-load rows are a separate 366 g condition**, measured with the object
  grasped rather than thread-suspended, at n=3. They are not part of the 100 g–1 kg
  sweep below, which predates the tegrastats logger.
- **Sustained actuator load costs the compute module essentially nothing**: 51.54 °C
  against active-idle's 51.1 °C, 5.475 W against 5.437 W, GPU never leaving its
  306 MHz floor. Reproducibility 0.25 °C on peak and 0.017 W on power.
- **VDD_IN is the Jetson's own rail; battery drain is the whole system.** Holding a
  pose adds +0.038 W of compute draw but +0.41 %/h of battery drain — that gap is
  the servo bus, which never appears in VDD_IN. Do not read 5.475 W as system power.
- **Inference and a loaded hold do not interfere.** Inference ran 999.1 ms median
  combined vs 1006.6 ms alone (0.7 % faster, within noise); the servo bus logged
  1738 samples at exactly 1 Hz with no gaps while the camera shared the same USB
  controller. Predicted 60.41 °C / 9.53 W from the components before measuring;
  measured 61.09 °C / 9.75 W.
- **Combined throttle = N** by Phase 2's three criteria: latency drifted −0.52 %
  across 1859 passes (it got *faster*), the GPU reached its full 1020 MHz ceiling,
  and 61.1 °C is ~25 °C below onset. A naive first-third/last-third clock comparison
  suggests throttling, but that detects DVFS tracking demand, not thermal limiting.
- **Battery drain resolution.** SoC is whole-percent; over ~26 min one 1 % step is
  worth ~2.4 %/h. The 4.59–6.19 spread across the three runs is smaller than one
  step, so they are consistent.

### A2 supplementary — sustained actuator load sweep (beyond protocol scope)

Bimanual hold of a fixed reference pose, load suspended on a thread from the right
arm; left arm is the unloaded paired control. Measured on `right_shoulder_lift`.
1 count = 0.0879° = 0.285 mm at the 18.6 cm moment arm.

| Load | Settle | Cur mean | Cur max | Load mean | Peak R (°C) | Peak L (°C) | ΔT gap | Drift | Minutes |
|---|---|---|---|---|---|---|---|---|---|
| 100 g | +18 | 1.5 | 2 | 50 | 37 | 39 | −2 | +0 | 30.0 |
| 200 g | +26 | 2.9 | 4 | 67 | 39 | 39 | +0 | +1 | 30.0 |
| 300 g | +35 | 4.5 | 7 | 86 | 41 | 38 | +3 | +7 | 30.0 |
| 400 g | +43 | 6.4 | 9 | 103 | 43 | 38 | +5 | +2 | 30.0 |
| 500 g | +52 | 9.9 | 14 | 129 | 48 | 39 | +9 | +8 | 30.0 |
| 600 g | +79 | 14.0 | 19 | 177 | 52 | 37 | +15 | +4 | 30.0 |
| 700 g | +80 | 20.2 | 25 | 186 | 60 | 38 | +22 | +10 | 30.0 |
| 800 g | +91 | 25.6 | 33 | 217 | **65** | 38 | +27 | +36 | **27.8** ⛔ |
| 900 g | +92 | 29.3 | 43 | 254 | **65** | 35 | +30 | +42 | **22.1** ⛔ |
| 1000 g | +101 | — | — | 216→298 | 61 | — | — | +707 | **13.9** ✗ |
| 1000 g (retry) | +108 | — | — | 216→258 | 55 | — | — | +727 | **8.8** ✗ |

⛔ stopped at the 65 °C ceiling — endurance point.  ✗ mechanical failure, not thermal.

**Endurance, normalised to a 35 °C start** (every ceiling run crosses 35 °C):

| Load | 35 → 65 °C |
|---|---|
| ≤700 g | >30 min (never reached ceiling) |
| 800 g | 26.4 min |
| 900 g | 19.6 min |
| 1000 g | **n/a — fails mechanically first** |

### Headline findings

1. **Thermally limited up to 900 g, mechanically limited at 1 kg.** At 700 g the
   shoulder sits at 43% of its torque budget but 92% of the 65 °C ceiling. At
   1000 g the joint slips before reaching the ceiling.
2. **The 1 kg failure is progressive, not a fixed threshold.** Two trials under
   matched conditions: slip at 13.7 min / 61 °C / load 298, then 8.7 min / 54 °C /
   load 258. Earlier, cooler, and at lower load each time, with settle degrading
   +101 → +108. Load before each slip was only 57–66% of the configured
   `Torque_Limit` (450), so the torque limit was **not** the cause — saturation to
   450 is a *consequence* of the position error after the drop.
3. **Discrete slip is distinguishable from creep.** Jumps >5 counts in one 1 Hz
   sample appear only at ≥800 g and account for 67% / 48% / 96% of total drift at
   800 / 900 / 1000 g. True creep is modest and gradual: +10, +12, +22, +25.
4. **Supply source materially changes results.** Same 800 g load, powerbank
   charging vs on battery: peak 63 → 65 °C, duration 30.0 → 27.8 min, current
   23.6 → 25.6, drift +19 → +36. Never run on the charger.

---

## B1 — Dynamic payload under motion

**NOT RUN.**

| Motion profile | Realized accel (m/s²) | Max mass, no slip (g) | First-slip mass (g) | n |
|---|---|---|---|---|
| Base 0.25 m/s² | — | — | — | — |
| Base 0.5 m/s² | — | — | — | — |
| Base 1.0 m/s² | — | — | — | — |
| Arm slew (high) | — | — | — | — |

Gates and deviations to record:

- Base has never been driven; `slew_payload_test.py` is untested.
- **Gated on shoulder-joint health** after the 1 kg mechanical failures.
- Protocol asks for **IMU** acceleration; this platform has a **D435 without IMU**.
  Use wheel-encoder odometry and state the substitution.
- B1 requires an actual **grasp** (thread suspension is invalid — slip is the
  measurement). The gripper already failed to close on a 475 g can, so the
  achievable range may be bounded by grasp geometry rather than dynamics.

---

## C1 — End-to-end learned-policy mobile manipulation

**NOT RUN — stack-blocked.**

| Stage | Attempts | Successes | Failures | Failure share (%) |
|---|---|---|---|---|
| Navigate | — | — | — | — |
| Perceive | — | — | — | — |
| Pick / grasp | — | — | — | — |
| Place | — | — | — | — |
| **Overall** | — | — | — | — |

RTAB-Map and Nav2 are **not installed**; this platform runs FAST-LIO + Livox.
Also requires a task-specific trained checkpoint and 100 trials.

---

## C2 — Inference optimization & control-rate characterization

### Stage 1 — Latency characterization ✅ n = 3, 60 s each

Baseline: `num_steps=10`, `chunk=50` (the checkpoint's native `chunk_size`), DVFS,
no graph capture. Each factor swept independently from that baseline.

| Factor | Level | Median e2e (ms) | Replan (Hz) | Control (Hz) |
|---|---|---|---|---|
| **Denoising steps** | 10 | 988.0 ±34.5 | 1.01 | 50.5 |
| | 5 | 617.6 ±15.8 | 1.62 | 81.1 |
| | 3 | 449.2 ±11.8 | 2.23 | 111.4 |
| | 2 | 363.2 ±0.8 | 2.75 | **137.7** |
| **Action chunk** | 5 | 995.8 ±0.6 | 1.00 | 5.0 |
| | 25 | 984.0 ±17.5 | 1.02 | 25.4 |
| | 50 | 988.0 ±34.5 | 1.01 | 50.5 |
| **Clock policy** | DVFS | 988.0 ±34.5 | 1.01 | 50.5 |
| | jetson_clocks | 923.5 ±5.0 | 1.08 | 54.1 |
| **Graph capture** | off | 988.0 ±34.5 | 1.01 | 50.5 |
| | `torch.compile` | 1013.3 ±28.7 | 0.99 | 49.3 |

**Fit: T = 216 ms fixed + steps × 77.7 ms** (max residual 13 ms over 4 levels).

Findings:

- **Chunk size is free.** 995.8 / 984.0 / 988.0 ms at chunk 5 / 25 / 50 —
  statistically identical latency — while control rate scales 5.0 → 25.4 → 50.5 Hz.
  A 10× control-rate gain at zero cost, purely from consuming actions the model
  already computes. Never exceed the checkpoint's native `chunk_size` (50).
- **Denoising steps are the only lever that matters**, and the relationship is linear.
- **`torch.compile(mode="reduce-overhead")` does not help: −2.6%, slightly worse
  than eager**, with `compiled=True` verified on all three reps (not a silent
  fallback). CUDA graphs do not touch the 216 ms floor.
- **`jetson_clocks` buys only 6.5%**, confirming the fixed cost is CPU-side dispatch
  that neither clock pinning nor graph capture can reach.

### Stage 2 — Thermal cost of the selected configuration ✅ 30 min

Config: `num_steps=2`, `chunk=50`, DVFS. 5012 forward passes.
Latency reproduced at **358.8 ms** vs Stage 1's 363.2 ms (1.2% agreement).

| Metric | A2 baseline (steps=10) | steps=2, chunk=50 | Δ |
|---|---|---|---|
| Peak GPU | 59.97 °C | **66.6 °C** | **+6.6 °C** |
| Mean GPU | 57.33 °C | 63.9 °C | +6.6 |
| GPU clock mean | 426 MHz | 751 MHz | +325 (+76%) |
| VDD_IN mean | 9.49 W | **12.69 W** | **+3.20 W (+34%)** |
| GPU util mean | 23–30% | 46% | — |
| Control rate | 50.5 Hz | 139.3 Hz | **2.76×** |
| **Throttle** | N | **Y (mild)** | — |

**Throttle detail:** GPU clock averaged 762 MHz over the first third and 694 MHz
over the last — a sustained ~9% reduction as temperature climbed. Report as
"Y (mild, ~9% clock reduction late in run)", not a bare yes.

**The speedup is not thermally free.** Fewer denoising steps means *more* forward
passes per second, each paying the full 216 ms fixed dispatch cost, so DVFS raises
the clock 76% and power with it.

**But efficiency roughly doubles:** 5.3 Hz/W at baseline (chunk-50 equivalent) vs
**11.0 Hz/W** at steps=2 — 2.76× the control rate for 1.34× the power. Both framings
belong in the paper: absolute power matters for untethered runtime, efficiency
matters for thermal budget.

### Stage 3 — Task-success validation ❌ NOT RUN

| Config | Attempts | Successes | Success rate | Indistinguishable from 10-step baseline? |
|---|---|---|---|---|
| steps=10 (baseline) | — | — | — | — |
| steps=5 | — | — | — | — |
| steps=3 | — | — | — | — |
| steps=2 | — | — | — | — |

**This gates every Stage 1 result.** Per the protocol, a latency reduction is only
reportable if success is statistically indistinguishable from the 10-step baseline.
Until this runs, **137.7 Hz at steps=2 is a measured latency, not a demonstrated
capability**. Requires ≥20 grasps per config on the standard object set.
