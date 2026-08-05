# A2 — State and Procedure (handoff)

Last updated 2026-08-04, mid-sweep at 800 g.

---

## 1. Where things stand

| Phase | Status |
|---|---|
| **A2 Phase 1 — active idle** | ✅ **n = 3 complete** |
| **A2 Phase 2 — peak inference** | ✅ **n = 3 complete** |
| **A2 Phase 3 — sustained actuator load** | 🔄 **8 of 10 loads done (100–800 g)** |
| C2 — inference optimisation | protocol drafted, Stage 1 preliminary (n=1) |
| A1, B1, C1, D1 | not started |

**Remaining in Phase 3:** 900 g, 1000 g. Both expected to abort at the ceiling, so both are endurance points rather than fixed 30-min runs.

---

## 2. Phase 3 procedure — follow exactly

```bash
ssh -t xle@10.0.0.197 'cd ~/xlerobot-pro && ~/.venvs/xlerobot-pro/bin/python \
  diagnostics/hold_pose_thermal.py --minutes 30 \
  --out results/A2/actuator_load/load_900g'
```

1. ENTER when clear → arms ramp to the saved pose over 6 s
2. **Only the right gripper releases** → hang the weight **on a thread**
3. ENTER → 3 s settle → logging starts
4. Type `bat NN` to log battery (stdin only — never write annotations.csv externally)

**Invariants that make the sweep comparable:**

- **Reference pose** lives in `calibration/hold_pose.json` and is replayed every
  run. Do NOT delete it — recapturing changes the moment arm and breaks
  comparability with all earlier loads.
- **Load is SUSPENDED FROM A THREAD**, not grasped. Removes grip force as a
  variable and removes the gripper's grasp ceiling (a 475 g can failed because
  the jaws could not close on it). Gripper drift reads 0 in every run, which
  confirms the jaws carry nothing.
- **Left arm is the unloaded paired control.** Report the within-run GAP between
  right and left, not absolute rise — baselines drifted 32–38 °C across the
  sweep and the gap normalises that out.
- **Cool to ~33–35 °C on `right_shoulder_lift` (ID 8) before each run.** From a
  60 °C peak this takes 20–25 min. Starting warm shortens time-to-ceiling and
  biases the endurance numbers.
- **Powerbank OFF the charger.** Pass-through charging changes the supply the
  servos see, so the run is not comparable with the battery-only 100–700 g runs.
  **The tell is a flat `bat` annotation** — a 30-min run normally costs ~3 %
  (500 g: 67→63, 600 g: 61→58), so an unchanging value means it is charging.
  Cost one full 800 g run; kept as `load_800g_CHARGING/` for comparison.
  59 % is enough for three more runs, so there is no reason to top it up mid-sweep.
- **Ambient 24.4 °C (76 °F)** throughout. Re-read and record if it changes.

---

## 3. Results so far

Recomputed identically from the raw CSVs (2026-08-04), so every column is
directly comparable. Current is given as **mean and max** — the old table's
single "Current" figure was the max.

| Load | Settle | Cur mean | Cur max | Load mean | Peak R | Peak L | ΔT gap | Drift | Minutes |
|---|---|---|---|---|---|---|---|---|---|
| 100 g | +18 | 1.5 | 2 | 50 | 37 | 39 | −2 | +0 | 30.0 |
| 200 g | +26 | 2.9 | 4 | 67 | 39 | 39 | +0 | +1 | 30.0 |
| 300 g | +35 | 4.5 | 7 | 86 | 41 | 38 | +3 | +7 | 30.0 |
| 400 g | +43 | 6.4 | 9 | 103 | 43 | 38 | +5 | +2 | 30.0 |
| 500 g | +52 | 9.9 | 14 | 129 | 48 | 39 | +9 | +8 | 30.0 |
| 600 g | +79 | 14.0 | 19 | 177 | 52 | 37 | +15 | +4 | 30.0 |
| 700 g | +80 | 20.2 | 25 | 186 | 60 | 38 | +22 | +10 | 30.0 |
| 800 g | +91 | 25.6 | 42 | 217 | **65** | 38 | **+27** | **+36** | **27.8** ⛔ |

⛔ = stopped itself at the 65 °C ceiling. **800 g is the first endurance point:
27.8 min sustained hold before the thermal limit.** Reached 60 °C at 21.7 min,
64 °C at 24.9, then took 2.9 min to cross the last degree.

### Supply source materially changes the result

The 800 g run was done twice — once accidentally with the powerbank on the
charger, once on battery (`load_800g_CHARGING/` vs `load_800g/`), from near
identical 34/33 °C starts. This is the cleanest evidence that the invariant
matters:

| | Charging | Battery |
|---|---|---|
| Peak temp | 63 °C | **65 °C** (ceiling) |
| Duration | 30.0 min | **27.8 min** |
| Current mean | 23.6 | **25.6** |
| Drift | +19 | **+36** |

On battery the joint drew ~8 % more current for the same torque and crept nearly
twice as far. Consistent with a sagging supply needing more current for the same
mechanical work, and more I²R heating with it. **Never run on the charger** — it
flatters every number.

Measured on `right_shoulder_lift` (the load-bearing joint at this pose)
against `left_shoulder_lift` as control. 1 count = 0.0879° = 0.285 mm at the
18.6 cm moment arm.

### Headline finding

**The platform is thermally limited, not torque limited.** At 700 g the shoulder
sits at **43 % of its torque budget but 92 % of the 65 °C ceiling**. Torque
extrapolates to only ~67 % at 1 kg; temperature reaches the ceiling far sooner.

### Secondary findings

- **Current is the most trustworthy column.** It depends only on torque, not on
  thermal history, so it is immune to the 32–38 °C baseline drift. 2 → 25 across
  the sweep, monotonic.
- **Creep is negligible below 500 g** (0–2 counts ≤ 0.57 mm over 30 min), becomes
  clear at 700 g (+10 counts = 2.9 mm), and **runs away at 800 g** (+36 counts =
  10.3 mm in 27.8 min — 3.6× the 700 g figure for 14 % more load, and still
  climbing when the run aborted). Creep is the steepest-responding metric in the
  whole sweep and is probably the real service limit, not temperature.
- Peak temperature rose ~4 °C/100 g to 600 g and ~8 °C/100 g to 700 g, but the
  column is censored at 65 °C from 800 g up — use time-to-ceiling above that.

### Projection for the remaining loads

**Peak temperature stops being a meaningful column at 800 g and above** — it is
censored at the 65 °C ceiling by construction. Time-to-ceiling replaces it.

Scaling from the measured 800 g point, assuming ohmic heating (rate ∝ I²) and
current continuing to rise ~2.9 per 100 g:

| Load | Cur mean | Time to 65 °C |
|---|---|---|
| 800 g | 25.6 | **27.8 min — measured** |
| 900 g | ~28.5 | ~22 min |
| 1000 g | ~31.4 | ~18 min |

Treat these as order-of-magnitude only: they ignore that convective loss also
grows with temperature, which biases them short.

From 800 g up, **every run becomes an endurance measurement** — the script stops
itself at the ceiling and the elapsed time is the result. Report as:

> Sustained-hold duration before reaching the 65 °C ceiling: ≥30 min at ≤700 g,
> declining with load above that.

Keep `--minutes 30` for all of them; the ceiling ends the run, not the clock.

---

## 4. Script behaviour and hard-won gotchas

### `diagnostics/hold_pose_thermal.py`

- **`target` and `baseline` are different columns and mean different things.**
  `target` = commanded position (the reference pose). `baseline` = where the arm
  actually settled at t=0. **Drift = position − baseline. Tracking error =
  position − target.** Do not confuse them.
- **NEVER re-baseline `Goal_Position` to the settled position.** A position-mode
  servo applies torque proportional to position error; zeroing that error makes
  it stop resisting gravity and the arm visibly drops a few mm. This was a real
  bug — the commanded pose must stay fixed, only the analysis baseline moves.
- **Release is a gradual torque bleed-off**, `TORQUE_RELEASE_SECONDS = 8.0`,
  stepping `Torque_Limit` 450 → 0 over 24 increments. `disable_torque` alone
  drops a loaded arm. Verified good at 700 g. Runs on every exit path
  (completion, ceiling abort, Ctrl-C, error) via `finally`.
- **`--max-temp` defaults to `SERVO_TEMP_CEILING_C` (65) from
  `firmware_limits.py`.** Firmware self-protects near 70 °C; stopping at 65
  leaves margin for a controlled descent.
- **`MAX_SETTLE_OFFSET = 150` hard-aborts a bad pose.** Loaded joints legitimately
  sag tens of counts (+88 at 800 g); hundreds means the arm never reached the
  reference geometry. The run stops *before* logging, prints the offending joints,
  removes the empty output directory, and exits through the gentle release — so
  there is no half-run to clean up and nothing to misread later.
- **`Goal_Position` is re-asserted just before the settle**, to clear a latched
  overload while it can still be recovered. See the trap below.
- Setup writes use `num_retry=5`. The bus occasionally returns a corrupted
  status packet and lerobot's `write` defaults to a single attempt.

### Operational traps

- **The servos latch into overload protection and do not self-recover.**
  Registers on every arm servo: `Overload_Torque` 80 %, `Protection_Time` 200
  (= 2.0 s), `Protective_Torque` 20, `Protection_Current` 310, against
  `Torque_Limit` 450 / `Max_Torque_Limit` 1000. Hold a servo above the threshold
  for 2 s and its output drops to 20 % **and stays there** — the arm collapses,
  which reads as a sudden mechanical failure but is the servo protecting itself.
  **A fresh `Goal_Position` write clears the latch.** This bit once while hanging
  800 g: the shoulder collapsed 1195 counts (~105°), the baseline was captured at
  the collapsed position, and it only recovered at t=1 when logging began and
  re-issued the goal. Hence the re-assert before the settle, and the
  `MAX_SETTLE_OFFSET` guard as a backstop.
- **A pose guard cannot catch an unloaded arm.** In that same run the arm held a
  perfectly good pose afterwards at current 9 / load 142 — 400 g-territory — because
  the load was not actually hanging on it. **Check current once holding:** it should
  track the table (~28–30 at 800 g). Geometry and load are independent failures.
- **Never delete a directory a running process has open.** Its writes go to an
  unlinked inode — invisible to `ls`, lost when the process exits. Recover while
  the process lives via `cp /proc/<pid>/fd/<n> <dest>`. This happened once and
  cost a near-miss on 14 000 rows.
- **`pgrep -f hold_pose_thermal` gives false positives** by matching my own
  polling shells. Use `ps -eo pid,args | grep "[h]old_pose_thermal.py" | grep -v "bash -c"`.
- **Bumps and knocks appear as discrete multi-count steps** in drift against an
  otherwise flat signal — a cat was located to the second at t=2647 s, a 5-count
  jump. If something is knocked mid-run, say so rather than restarting: a 52 s
  exclusion usually beats losing 30 minutes.
- **A failed ramp looks like saturation but isn't.** One 400 g attempt settled
  246 counts short with current 84 / load 410; the clean rerun settled +43 with
  current 8 / load 102. If a settle value comes back in the hundreds, the ramp
  failed — restart rather than run on bad geometry. See
  `load_400g_FAILED_SETUP/` for what that looks like.

---

## 5. Data layout

```
results/A2/
├── STATE.md                      this file
├── peak_inference/               n=3, SUMMARY_n3.md
├── active_idle/                  n=3, SUMMARY.md
├── actuator_load/
│   ├── load_100g … load_700g/    servo_telemetry.csv, annotations.csv
│   └── load_400g_FAILED_SETUP/   example of a failed ramp
└── optimisation_sweep/           11 configs, C2 preliminary
```

`servo_telemetry.csv` is 1 Hz × 17 servos, long format:
`elapsed_s, timestamp, bus, motor, position, temp_c, current, load, target, baseline`

**Back up after every session** — the Jetson's SD card has accumulated ext4
errors from power loss:

```bash
rsync -av xle@10.0.0.197:xlerobot-pro/results/A2 ~/Desktop/Workspace/xlerobot-pro-data/
```

---

## 6. Uncommitted repo changes

All on `restructure/project-organization`, none pushed.

| File | Change |
|---|---|
| `README.md` | PyTorch index `.dev` → `.io` (old domain is NXDOMAIN) |
| `pyproject.toml` | torch `<2.12.0`, torchvision `<0.27.0` — old ceilings were below the oldest available wheel |
| `src/xlerobot_pro/ik.py` | `IndexError` on unreachable IK targets |
| `src/xlerobot_pro/config.py` | `CAPTURE_DIR` — writer and reader disagreed on the capture path |
| `src/xlerobot_pro/firmware_limits.py` | new Thermal section: `SERVO_TEMP_CEILING_C`, `TORQUE_RELEASE_SECONDS` |
| `diagnostics/hold_pose_thermal.py` | new — A2 Phase 3 |
| `diagnostics/slew_payload_test.py` | new — B1 arm-slew, untested |
| `protocols/C2_inference_optimization.md` | new |

---

## 7. Next steps after Phase 3

1. **C2 Stage 1** — inference latency sweep at n=3. Fully unattended, ~2 h, no
   new equipment. Cheapest remaining work.
2. **D1 creep** — calendar-limited (24 h/run), so start early. Needs printed
   PLA/PETG/ASA coupons and a dial indicator. Note Phase 3 gives an *inferred*
   actuator-seat temperature only — no external probe was ever fitted.
3. **B1 payload under motion** — `slew_payload_test.py` exists but is UNTESTED.
   Gate is that the base has never been driven. Note the protocol asks for IMU
   acceleration; the platform has a **D435 (no IMU)** so use wheel-encoder
   odometry instead.
4. **A1 brownout** — needs bench supply + scope + rewiring to the old harness.
   ⚠️ The protocol says "Tri-Bus" but the robot is **2-bus**.
5. **C1 policy trials** — Nav2 / RTAB-Map are **not installed**; the platform has
   FAST-LIO + Livox instead. 100 trials is the largest effort of any test.
