## A2 Phase 3 — Sustained Actuator Load

Bimanual hold of a fixed reference pose. The load hangs on a thread from the right
arm; the left arm holds the identical pose unloaded as a paired control. All
measurements on `right_shoulder_lift` (the load-bearing joint at this pose).

**Conditions:** ambient 24.4 C; powerbank on battery, never charging; reference pose
replayed from file every run so the moment arm is identical across loads.

**Units:** 1 count = 0.0879 deg joint rotation = 0.285 mm vertical travel at the
18.6 cm moment arm. Torque is reported as a percentage of the configured
`Torque_Limit` (450 of a 1000 maximum), so 60% of limit is only 27% of what the
motor can produce.

**Definitions**

| Term | Meaning |
|---|---|
| Deflection (settle) | Static sag below the commanded position once motion stops. A position-mode servo holds by applying torque proportional to position error, so it rests wherever that torque balances the load. Direct measure of joint effort. |
| Drift | Total movement during the run, measured from the settled position. Combines slow creep and discrete slip. |
| Creep | The smooth component of drift. |
| Slip | A discrete jump of more than 5 counts within one 1 Hz sample. |
| dT gap | Peak temperature of the loaded joint minus the unloaded control joint. Removes ambient and chassis variation. |

---

### Table 1 — Static response

| Load (g) | Deflection (counts) | Deflection (mm) | Current | Torque | Torque (% of limit) |
|---|---|---|---|---|---|
| 100 | +18 | 5.1 | 1.5 | 50 | 11% |
| 200 | +26 | 7.4 | 2.9 | 67 | 15% |
| 300 | +35 | 10.0 | 4.5 | 86 | 19% |
| 400 | +43 | 12.3 | 6.4 | 103 | 23% |
| 500 | +52 | 14.8 | 9.9 | 129 | 29% |
| 600 | +79 | 22.5 | 14.0 | 177 | 39% |
| 700 | +80 | 22.8 | 20.2 | 186 | 41% |
| 800 | +91 | 25.9 | 25.6 | 217 | 48% |
| 900 | +92 | 26.2 | 29.3 | 254 | 56% |
| 1000 (trial 1) | +101 | 28.8 | 31.8 | 271 | 60% |
| 1000 (trial 2) | +108 | 30.8 | 33.2 | 247 | 55% |

---

### Table 2 — Thermal response

| Load (g) | Peak loaded joint (C) | Peak control joint (C) | dT gap (C) | Duration (min) | Outcome |
|---|---|---|---|---|---|
| 100 | 37 | 39 | -2 | 30.0 | completed 30 min |
| 200 | 39 | 39 | +0 | 30.0 | completed 30 min |
| 300 | 41 | 38 | +3 | 30.0 | completed 30 min |
| 400 | 43 | 38 | +5 | 30.0 | completed 30 min |
| 500 | 48 | 39 | +9 | 30.0 | completed 30 min |
| 600 | 52 | 37 | +15 | 30.0 | completed 30 min |
| 700 | 60 | 38 | +22 | 30.0 | completed 30 min |
| 800 | 65 | 38 | +27 | 27.8 | stopped at 65 C ceiling |
| 900 | 65 | 35 | +30 | 22.1 | stopped at 65 C ceiling |
| 1000 (trial 1) | 61 | 37 | +24 | 13.9 | mechanical failure |
| 1000 (trial 2) | 55 | 35 | +20 | 8.8 | mechanical failure |

---

### Table 3 — Positional stability over time

| Load (g) | Drift (counts) | Drift (mm) | Creep (counts) | Slip (counts) | Slip events |
|---|---|---|---|---|---|
| 100 | +0 | 0.0 | +0 | +0 | none |
| 200 | +1 | 0.3 | +1 | +0 | none |
| 300 | +7 | 2.0 | +1 | +6 | +6 at 28.8 min |
| 400 | +2 | 0.6 | +2 | +0 | none |
| 500 | +8 | 2.3 | +8 | +0 | none |
| 600 | +4 | 1.1 | +4 | +0 | none |
| 700 | +10 | 2.8 | +10 | +0 | none |
| 800 | +36 | 10.3 | +12 | +24 | +7 at 22.4 min; +17 at 25.6 min |
| 900 | +42 | 12.0 | +22 | +20 | +12 at 5.1 min; +8 at 6.1 min |
| 1000 (trial 1) | +707 | 201.5 | +25 | +682 | +24 at 4.4 min; +658 at 13.7 min |
| 1000 (trial 2) | +739 | 210.6 | +12 | +727 | +727 at 8.7 min |

---

### Table 4 — Endurance to the 65 C thermal ceiling

Normalised to a 35 C start so runs beginning at different temperatures compare
directly. Every run that reached the ceiling passes through 35 C on the way up.

| Load (g) | Start temp (C) | Time 35 -> 65 C (min) |
|---|---|---|
| 100 | 37 | > 30 (ceiling not reached) |
| 200 | 36 | > 30 (ceiling not reached) |
| 300 | 37 | > 30 (ceiling not reached) |
| 400 | 37 | > 30 (ceiling not reached) |
| 500 | 33 | > 30 (ceiling not reached) |
| 600 | 36 | > 30 (ceiling not reached) |
| 700 | 33 | > 30 (ceiling not reached) |
| 800 | 33 | **26.4** |
| 900 | 29 | **19.6** |
| 1000 (trial 1) | 34 | n/a - fails mechanically first |
| 1000 (trial 2) | 35 | n/a - fails mechanically first |
