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

Register semantics (Feetech STS3215 firmware):

- ``Torque_Limit`` — integer 0-1000, fraction of stall torque in 0.1 %
  steps. Conversion for the STS3215: tau = 450 ~= 1.32 N*m,
  tau = 650 ~= 1.91 N*m.
- ``Acceleration`` — integer 0-254, ramp steepness (one unit ~= 8.7
  deg/s^2). Lower values give softer ramps and smaller inrush-current
  spikes; this is the primary speed governor for position moves.
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

#: Maximum acceleration for the arm motors ("Soft" ramp), chosen to
#: minimize inrush spikes during manipulation.
ARM_ACCELERATION = 40

#: Maximum speed for the arm motors (``Maximum_Velocity_Limit``
#: register). Caps how fast a position move may run regardless of the
#: commanded trajectory.
ARM_MAX_VELOCITY = 100

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
