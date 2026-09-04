"""System-wide firmware saturation limits for XLeRobot-Pro.

This is the single source of truth for the platform's maximum torque,
acceleration, and speed settings (Table III of the XLeRobot-Pro paper).
The limits bound how hard and how fast every motor can move so that each
power bus stays inside its fuse rating and motor transients can never
brown out the Jetson.

═══════════════════════════════════════════════════════════════════════
 USER-TUNABLE: edit the constants below to change the system-wide
 maximums. They are applied automatically everywhere motors are
 configured — by the XLerobot robot classes at connect()/configure()
 time and by the vision demos before any motion.
═══════════════════════════════════════════════════════════════════════

BUS MAXIMA (Table III of the paper). Each bus is permitted at most 98 % of its
fuse. The caps below are set so that Eq. (2) of the paper -- the loaded joints at
the cap, the rest at no-load current -- stays under that maximum:

    Bus Group            tau   accel   Max draw   Eq. (2) worst case   Measured
    Bus A (wheels/neck)  650   20      9.84 A     9.13 A (5 x 1.83 A)  --
    Bus B (arms)         450   40      4.90 A     4.44 A (2 x 1.32 + 10 x 0.18)   1.41 A per arm, <= 2.8 A both
    Jetson Orin          n/a   n/a    <2.10 A     --                   --

Bus B was measured on 2 Sep 2026 with an inline ammeter (FNIRSI 2C53T, MAX hold)
on one arm's supply lead during five worst-case simultaneous six-joint moves at
this envelope: 1.41 A peak, 0.15 A idle (xlerobot-pro-data, A1/bus_b_peak_left_run4).
Bus A has not been measured. The residual between the Eq. (2) worst case and the
maximum (0.71 A on Bus A, 0.46 A on Bus B) is the inrush allowance. Raising torque
or acceleration on either bus spends that allowance; the limits are the platform's
envelope, not defaults awaiting tuning.

Consequences worth knowing before trying to design around them:

- Base acceleration is capped at 0.152 m/s^2. WHEEL_NECK_ACCELERATION = 20 units
  x 8.7 deg/s^2 x 0.05 m wheel radius = 0.152 m/s^2, confirmed by measurement
  (0.151 and 0.136 m/s^2 realized when commanding 0.25 and 0.5). Commanding more
  does not produce more.
- Under a 366 g payload, right_shoulder_lift realizes ~1 % of a commanded slew
  amplitude at tau=450; right_elbow_flex reaches ~29 %. Choose the joint rather
  than raising the limit.

Register semantics (Feetech STS3215 firmware):

- ``Torque_Limit`` — integer 0-1000, fraction of stall torque in 0.1 %
  steps. Conversion for the STS3215: tau = 450 ~= 1.32 N*m,
  tau = 650 ~= 1.91 N*m.
- ``Acceleration`` — integer 0-254, ramp steepness (one unit ~= 8.7
  deg/s^2). Lower values give softer ramps and smaller inrush-current
  spikes; this is the primary speed governor for position moves.
- ``Present_Temperature`` — integer degrees C, the servo's own internal
  sensor. The firmware protects itself near 70 C; the ceiling below
  stops well short of that so a run ends on our terms, not the
  firmware's.
"""

# ─────────────────────────────────────────────────────────────────────
# Bus A — wheels + neck (10 A fuse, PDU USB-C2 output)
# ─────────────────────────────────────────────────────────────────────

#: Maximum torque for the wheel and neck motors.
#: Firmware units 0-1000; 650 ~= 1.91 N*m on the STS3215.
#: Raise only if the 10 A Bus A fuse and PDU output can take the load.
WHEEL_NECK_TORQUE_LIMIT = 650

#: Maximum acceleration for the wheel and neck motors ("Medium" ramp).
#: Firmware units 0-254; lower = gentler starts = smaller current spikes.
WHEEL_NECK_ACCELERATION = 20

#: Maximum raw wheel velocity command (firmware ticks) for the mobile
#: base. Goal_Velocity commands above this are scaled down uniformly,
#: capping the robot's top driving speed.
WHEEL_MAX_RAW_SPEED = 3000

# ─────────────────────────────────────────────────────────────────────
# Bus B — arms (5 A fuse, PDU DC car outlet)
# ─────────────────────────────────────────────────────────────────────

#: Maximum torque for the arm motors.
#: Firmware units 0-1000; 450 ~= 1.32 N*m on the STS3215 — enough for
#: the rated 1 kg payload while keeping a dual-arm stall inside the
#: 5 A Bus B fuse.
ARM_TORQUE_LIMIT = 450

#: RETIRED — do not use. Kept only so existing imports fail loudly rather than
#: silently reverting to the default.
#:
#: This was added to let shoulder_lift follow a slew trajectory under payload,
#: justified against an estimated ~1.3 A slew-case draw. Bus B's permitted maximum
#: is 4.90 A against a 5 A fuse and Eq. (2) puts the tau=450 worst case at 4.44 A
#: (Table III); raising two joints to 650 adds roughly 0.6 A to that worst case,
#: which spends the whole inrush allowance. B1 arm-slew
#: runs recorded with boosted=true in their run_info.json were collected outside the
#: documented envelope and should be reported as such, or repeated at tau=450.
SLEW_TORQUE_LIMIT = None

#: Maximum acceleration for the arm motors ("Soft" ramp), chosen to
#: minimize inrush spikes during manipulation.
ARM_ACCELERATION = 40

#: Maximum speed for the arm motors (``Maximum_Velocity_Limit``
#: register). Caps how fast a position move may run regardless of the
#: commanded trajectory.
ARM_MAX_VELOCITY = 100

# ─────────────────────────────────────────────────────────────────────
# Thermal
# ─────────────────────────────────────────────────────────────────────

#: Temperature ceiling for sustained-load work, degrees C.
#: The STS3215 firmware trips its own overheat protection near 70 C. A
#: run should stop before that: a firmware trip is abrupt and gives no
#: controlled descent, whereas stopping here leaves margin to lower the
#: arms gently. Also the endurance measurement -- time to reach this
#: ceiling is what "how long can it hold X" means in practice.
SERVO_TEMP_CEILING_C = 65

#: Seconds over which to bleed torque away when releasing a loaded arm.
#: Cutting Torque_Limit to zero in one write drops the arm; stepping it
#: down lets gravity lower it against progressively weaker resistance.
TORQUE_RELEASE_SECONDS = 8.0

# ─────────────────────────────────────────────────────────────────────
# Power-on defaults (EPROM)
# ─────────────────────────────────────────────────────────────────────

#: ``Max_Torque_Limit`` EPROM ceiling — the value the firmware copies
#: into ``Torque_Limit`` at power-on, before the software limits above
#: are applied at configure() time. Must be >= both torque limits above.
MAX_TORQUE_EPROM = 800

# ─────────────────────────────────────────────────────────────────────
# Application helpers (used by robot classes and demos — no need to
# call these yourself unless you are writing a new control script)
# ─────────────────────────────────────────────────────────────────────


def apply_arm_limits(bus, motor_names) -> None:
    """Write the arm (Bus B) saturation limits to the given motors.

    Call with bus torque disabled (e.g. during ``configure()``); the
    caller re-enables torque afterwards.
    """
    for name in motor_names:
        bus.write("Torque_Limit", name, ARM_TORQUE_LIMIT)
        bus.write("Acceleration", name, ARM_ACCELERATION)
        bus.write("Maximum_Velocity_Limit", name, ARM_MAX_VELOCITY)


def apply_wheel_neck_limits(bus, motor_names) -> None:
    """Write the wheel/neck (Bus A) saturation limits to the given motors.

    Call with bus torque disabled (e.g. during ``configure()``); the
    caller re-enables torque afterwards.
    """
    for name in motor_names:
        bus.write("Torque_Limit", name, WHEEL_NECK_TORQUE_LIMIT)
        bus.write("Acceleration", name, WHEEL_NECK_ACCELERATION)
