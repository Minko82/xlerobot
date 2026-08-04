# A2 — State and Procedure (handoff)

Last updated 2026-08-04, mid-sweep at 800 g.

---

## 1. Where things stand

| Phase | Status |
|---|---|
| **A2 Phase 1 — active idle** | ✅ **n = 3 complete** |
| **A2 Phase 2 — peak inference** | ✅ **n = 3 complete** |
| **A2 Phase 3 — sustained actuator load** | 🔄 **7 of 10 loads done (100–700 g)** |
| C2 — inference optimisation | protocol drafted, Stage 1 preliminary (n=1) |
| A1, B1, C1, D1 | not started |

**Remaining in Phase 3:** 800 g, 900 g, 1000 g.

---

## 2. Phase 3 procedure — follow exactly

```bash
ssh -t xle@10.0.0.197 'cd ~/xlerobot-pro && ~/.venvs/xlerobot-pro/bin/python \
  diagnostics/hold_pose_thermal.py --minutes 30 \
  --out results/A2/actuator_load/load_800g'
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
- **Ambient 24.4 °C (76 °F)** throughout. Re-read and record if it changes.

---

## 3. Results so far

| Load | Current | Load/450 | ΔT gap | Peak °C | Drift (counts) | Settle |
|---|---|---|---|---|---|---|
| 100 g | 2 | — | −2 | 37 | 0 | +18 |
| 200 g | 4 | — | +1 | 39 | +1 | ~+18 |
| 300 g | 7 | — | +2 | 41 | +1 | ~+18 |
| 400 g | 9 | 102 (23%) | +4 | 43 | +2 | +43 |
| 500 g | 14 | 134 (30%) | +9 | 48 | +8 | +52 |
| 600 g | 19 | 180 (40%) | +13 | 52 | +4 | +79 |
| 700 g | 25 | 194 (43%) | +21 | **60** | **+10** | +80 |

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
- **Creep is negligible below 500 g** (0–2 counts ≤ 0.57 mm over 30 min) and only
  becomes clear at 700 g (+10 counts = 2.9 mm, still climbing at 30 min).
- Peak temperature rises ~4 °C/100 g up to 600 g, then **~8 °C/100 g** — it
  accelerates.

### Projection for the remaining loads

| Load | Projected peak | Expectation |
|---|---|---|
| 800 g | ~68 °C | **hits the 65 °C ceiling, aborts ~min 18–22** |
| 900 g | ~76 °C | aborts sooner |
| 1000 g | ~84 °C | aborts soonest |

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
- Setup writes use `num_retry=5`. The bus occasionally returns a corrupted
  status packet and lerobot's `write` defaults to a single attempt.

### Operational traps

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
