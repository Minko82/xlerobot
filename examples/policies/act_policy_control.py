#!/usr/bin/env python3
"""Run a trained ACT checkpoint on one XLeRobot-Pro arm.

The gap this closes: ``lerobot-train`` produces an ACT checkpoint, and until now
nothing in this repository could execute one. ``examples/policies/`` had runners
for the diffusion policy and SmolVLA only, and ``scripts/policy_trials.py`` takes
``--cmd`` as a *required* argument -- so there was no way to start a trial block
at all. Sec. VI-E of the paper depends on this file existing.

Why ACT rather than the diffusion policy. The diffusion runner here is vision-only
by construction: ``ConditionalUNet1D.forward`` takes a single conditioning vector
built from the visual encoder, and joint positions are read and then dropped. ACT
is state+vision by default, which is both the stronger result and the honest one
for a paper whose thesis is that proprioceptive load matters.

CONTROL RATE, and why this prints three numbers rather than one. The paper quotes
a two-term latency model and a chunked control rate (``K=50`` giving 50.5 Hz).
Those are bench latencies. What a reviewer will ask for is the *realised* rate
under load, which is not the same thing once camera reads and bus writes are in
the loop, so this reports:

    inference    median ms inside predict_action alone
    loop         median ms for the whole observe -> predict -> send cycle
    realised     steps / wall seconds, the number Sec. VI-E should quote

``policy_trials.py`` prompts the operator for the step count after each trial, so
the final line is printed large and last, ready to be typed in.

The firmware envelope (Table III) is applied before anything moves, so a policy
cannot command the arm outside the limits the paper claims for it.

    python examples/policies/act_policy_control.py run \\
        --checkpoint outputs/act_bottle_pickplace/checkpoints/last/pretrained_model \\
        --duration 30

Ctrl-C stops early and bleeds torque away rather than cutting it, so the arm
settles instead of dropping.
"""

from __future__ import annotations

import argparse
import statistics
import sys
import time
from pathlib import Path

from lerobot.cameras.configs import ColorMode
from lerobot.cameras.realsense.configuration_realsense import RealSenseCameraConfig
from lerobot.configs.policies import PreTrainedConfig
from lerobot.datasets.utils import build_dataset_frame, combine_feature_dicts, hw_to_dataset_features
from lerobot.policies.act.modeling_act import ACTPolicy
from lerobot.policies.factory import make_pre_post_processors
from lerobot.policies.utils import make_robot_action
from lerobot.robots.so101_follower.config_so101_follower import SO101FollowerConfig
from lerobot.utils.constants import OBS_STR
from lerobot.utils.control_utils import predict_action
from lerobot.utils.robot_utils import busy_wait
from lerobot.utils.utils import get_safe_torch_device

from xlerobot_pro.firmware_limits import (
    ARM_ACCELERATION,
    ARM_MAX_VELOCITY,
    ARM_TORQUE_LIMIT,
    SERVO_TEMP_CEILING_C,
    TORQUE_RELEASE_SECONDS,
)

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "scripts"))
try:
    from xle_arms import ARM_IDS, SO101FollowerArm
except ImportError:  # running from the data checkout, where xle_arms sits alongside
    sys.path.insert(0, str(Path.home() / "xlerobot-pro-data" / "scripts"))
    from xle_arms import ARM_IDS, SO101FollowerArm

#: Stop if any joint reaches the ceiling mid-trial. A trial that cooks the
#: shoulder is not a policy result, and Sec. VI-E's whole point is to report
#: success *against* joint temperature rather than despite it.
TEMP_CHECK_EVERY = 30

#: Action keys starting with this are passed through unsmoothed.
GRIPPER_PREFIX = "gripper"


#: Attempts before giving up on the bus. lerobot's reads and writes default to
#: no retry, so a single dropped packet raises straight out of the control loop
#: and ends the trial. Measured on this arm, sync_read of all six motors fails
#: about 0.13% of the time -- roughly one failure per 900 reads, which is one
#: per 30 s of running at 30 Hz. Without retries most trials would not finish,
#: and a trial that dies mid-grasp is not a measurement of anything.
#:
#: The backoff doubles from 10 ms, so six attempts span ~310 ms.
BUS_RETRIES = 6
BUS_BACKOFF_S = 0.01


def harden_bus(bus, retries: int = BUS_RETRIES):
    """Make every read and write on this bus survive a dropped packet.

    Wrapping the bound methods on the instance covers lerobot's own internal
    ``self.sync_read(...)`` calls too, because an instance attribute shadows the
    class method. Applied before ``connect()`` so ``configure()`` is covered as
    well -- that path fails as ``Failed to write 'Lock' on id_=5``.

    This is mitigation, not a repair: see ``scripts/bus_watch.py`` in the data
    repository for locating the underlying fault.
    """
    def wrap(fn):
        def retrying(*a, **kw):
            for attempt in range(retries):
                try:
                    return fn(*a, **kw)
                except ConnectionError:
                    if attempt == retries - 1:
                        raise
                    time.sleep(BUS_BACKOFF_S * (2 ** attempt))
        return retrying

    for name in ("sync_read", "sync_write", "read", "write"):
        if hasattr(bus, name):
            setattr(bus, name, wrap(getattr(bus, name)))


def apply_envelope(robot) -> None:
    """Clamp the arm to Table III before the policy is allowed to command it."""
    names = list(robot.bus.motors)
    robot.bus.disable_torque(names)
    for n in names:
        robot.bus.write("Torque_Limit", n, ARM_TORQUE_LIMIT)
        robot.bus.write("Acceleration", n, ARM_ACCELERATION)
        robot.bus.write("Maximum_Velocity_Limit", n, ARM_MAX_VELOCITY)
    robot.bus.enable_torque(names)


def release_gently(robot, seconds: float = TORQUE_RELEASE_SECONDS, steps: int = 24) -> None:
    """Bleed torque to zero so the arm settles instead of dropping.

    Deliberately hard to interrupt. A Ctrl-C during the ramp used to raise
    straight out of ``time.sleep`` and abandon it partway -- and the ramp exists
    for exactly this moment, with the arm holding a pose under load. The
    protocol requires the bleed on every exit path including the autonomous one,
    so an impatient operator gets a *faster* bleed rather than none: the first
    interrupt shortens the remaining ramp fourfold instead of cancelling it.
    """
    names = list(robot.bus.motors)
    interrupted = False
    try:
        for i in range(steps - 1, -1, -1):
            for n in names:
                try:
                    robot.bus.write("Torque_Limit", n, int(ARM_TORQUE_LIMIT * i / steps))
                except Exception:
                    pass
            try:
                time.sleep((seconds / steps) / (4.0 if interrupted else 1.0))
            except KeyboardInterrupt:
                if not interrupted:
                    interrupted = True
                    print("\n  interrupt during torque bleed -- finishing it fast, "
                          "not skipping it")
    finally:
        for n in names:
            try:
                robot.bus.write("Torque_Limit", n, ARM_TORQUE_LIMIT)
            except Exception:
                pass


def load_policy(checkpoint: Path):
    """Config, weights and processors, all from the checkpoint -- no dataset needed.

    ``make_pre_post_processors`` reads the normalisation statistics saved beside the
    weights, which is what makes standalone inference possible: rebuilding them from
    a dataset would mean the dataset had to still exist, and had to be the same one.
    """
    cfg = PreTrainedConfig.from_pretrained(checkpoint)
    cfg.pretrained_path = str(checkpoint)
    # NOT make_policy(): it derives feature shapes from a dataset or a sim env and
    # raises "Either one of a dataset metadata or a sim env must be provided" when
    # given neither -- so it can never load a checkpoint standalone, which is the
    # whole point here. from_pretrained reads the shapes out of the checkpoint's
    # own config, which already records them.
    policy = ACTPolicy.from_pretrained(checkpoint)
    policy.eval()
    pre, post = make_pre_post_processors(cfg, pretrained_path=str(checkpoint))
    return policy, pre, post, cfg


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = p.add_subparsers(dest="command", required=True)
    r = sub.add_parser("run", help="Drive the arm from a trained checkpoint.")
    r.add_argument("--checkpoint", type=Path, required=True,
                   help="A pretrained_model directory written by lerobot-train.")
    r.add_argument("--duration", type=float, default=30.0, help="Seconds to run.")
    r.add_argument("--steps", type=int, default=None,
                   help="Stop after this many steps instead of on --duration.")
    r.add_argument("--port", default="/dev/xle_arms")
    r.add_argument("--arm", choices=list(ARM_IDS), default="left")
    r.add_argument("--robot-id", default=None, help="Defaults to <arm>_follower.")
    r.add_argument("--camera-serial", default="838212073725")
    r.add_argument("--width", type=int, default=640)
    r.add_argument("--height", type=int, default=480)
    r.add_argument("--fps", type=float, default=30.0, help="Target control rate, Hz.")
    r.add_argument("--task", default="pick up the bottle and place it on the other side",
                   help="Must match the task string the checkpoint was trained on.")
    r.add_argument("--n-action-steps", type=int, default=None, metavar="N",
                   help="Actions executed per inference, overriding the checkpoint. ACT "
                        "queues N actions then re-infers, so a large N means a long stretch "
                        "of open-loop motion and a big correction when the queue empties -- "
                        "that jump is what reads as twitchiness. Smaller N corrects more "
                        "often by less. Must not exceed chunk_size. Inference-time only: it "
                        "changes nothing about the trained weights.")
    r.add_argument("--smooth", type=float, default=1.0, metavar="A",
                   help="Exponential smoothing on the commanded joint targets: "
                        "cmd = A*action + (1-A)*previous_cmd. 1.0 (default) sends the "
                        "policy output unchanged, which is what every measurement so far "
                        "used. Lower values damp the step change that lands each time "
                        "ACT's queue empties -- the loop stalls ~100 ms for inference, the "
                        "servo holds its last goal, and the next command arrives as a jump. "
                        "0.3-0.5 is a reasonable place to start. Costs tracking lag, so "
                        "report the value used.")
    r.add_argument("--aim-offset", default=None, metavar="JOINT=UNITS",
                   help="Constant corrections added to the policy's commanded joints, "
                        "comma separated, e.g. 'shoulder_pan=-0.9'. This is for a "
                        "MEASURED bias in the policy's own output, not a tuning knob: "
                        "against glassbottle_pick_v3's own training frames the policy "
                        "predicts shoulder_pan with slope 1.037 and intercept +0.89, so "
                        "it tracks the bottle faithfully and simply aims ~0.9 units to "
                        "one side. Anything applied here changes what a reported success "
                        "rate means and belongs in the results. Re-derive it whenever the "
                        "scene changes -- an offset fitted under the wrong lighting or a "
                        "drifted rig will over-correct once those are fixed.")
    r.add_argument("--log-trajectory", type=Path, default=None, metavar="CSV",
                   help="Write per-step observed state and commanded action to CSV. "
                        "The only way to tell a policy that never commanded a motion "
                        "from one whose command was clamped, filtered or bled away on "
                        "exit -- the arm's final pose cannot distinguish those.")
    r.add_argument("--log-frames", type=Path, default=None, metavar="DIR",
                   help="Save the camera frame the policy saw every 30 steps as JPEG, "
                        "named by step. The trajectory CSV says where the arm closed; "
                        "only the frame says where the bottle was when it did.")
    r.add_argument("--temporal-ensemble", type=float, default=None, metavar="COEFF",
                   help="Infer every step and execute the exponentially weighted average of "
                        "every past chunk's prediction for the current step (ACT's own "
                        "temporal ensembling; the paper used 0.01). Replaces the "
                        "n_action_steps queue, so visual correction happens every step and "
                        "the start-pose dwell cannot lock the arm: chunks predicted earlier "
                        "already call for motion now.")
    r.add_argument("--overlay", type=Path, default=None, metavar="IMAGE",
                   help="Paste a fixed region of this 640x480 training frame onto every "
                        "live frame before inference. Kinesthetic demonstrations leave the "
                        "operator's hand on the wrist, in the wrist camera's view; the "
                        "policy learned to read it, and without it closes early and short.")
    r.add_argument("--overlay-region", default="200,480,0,260", metavar="Y0,Y1,X0,X1",
                   help="Pixel box of --overlay to paste. Default covers the gripper body "
                        "and hand at the bottom-left, leaving the moving jaw visible.")
    r.add_argument("--no-envelope", action="store_true",
                   help="Skip the Table III clamp. Only for the C1 unsized experiment.")
    args = p.parse_args()
    if args.robot_id is None:
        args.robot_id = f"{args.arm}_follower"

    if not args.checkpoint.exists():
        print(f"\n  No checkpoint at {args.checkpoint}\n"
              "  Train one first:\n"
              "    lerobot-train --policy.type=act --dataset.repo_id=local/bottle_pickplace \\\n"
              "        --output_dir=outputs/act_bottle_pickplace --policy.device=cuda\n",
              file=sys.stderr)
        return 1

    print(f"\n  loading {args.checkpoint}")
    policy, pre, post, cfg = load_policy(args.checkpoint)

    # Overriding after load, on the live config object, because ACT reads
    # n_action_steps at queue-refill time rather than at construction.
    if args.n_action_steps is not None:
        if args.n_action_steps > cfg.chunk_size:
            print(f"  --n-action-steps {args.n_action_steps} exceeds chunk_size "
                  f"{cfg.chunk_size}", file=sys.stderr)
            return 1
        policy.config.n_action_steps = args.n_action_steps
        cfg.n_action_steps = args.n_action_steps
        print(f"  n_action_steps overridden to {args.n_action_steps} "
              f"(chunk_size {cfg.chunk_size}) -- re-infers every "
              f"{args.n_action_steps / args.fps:.2f} s")
    if args.temporal_ensemble is not None:
        from lerobot.policies.act.modeling_act import ACTTemporalEnsembler
        policy.config.temporal_ensemble_coeff = args.temporal_ensemble
        cfg.temporal_ensemble_coeff = args.temporal_ensemble
        policy.temporal_ensembler = ACTTemporalEnsembler(args.temporal_ensemble, cfg.chunk_size)
        policy.config.n_action_steps = 1
        cfg.n_action_steps = 1
        print(f"  temporal ensembling coeff {args.temporal_ensemble:g}: inferring every step, "
              "n_action_steps forced to 1   -- RECORD THIS ALONGSIDE ANY SUCCESS RATE")
    device = get_safe_torch_device(cfg.device)
    chunk = getattr(cfg, "chunk_size", "?")
    n_act = getattr(cfg, "n_action_steps", "?")
    print(f"  ACT on {device}   chunk_size={chunk}  n_action_steps={n_act}")
    if isinstance(n_act, int) and isinstance(chunk, int) and n_act == chunk:
        print(f"  NOTE: one inference commits {n_act} open-loop steps "
              f"({n_act / args.fps:.1f} s at {args.fps:g} Hz). Sec. VI-E should say so "
              "next to the quoted control rate.")

    cfg_robot = SO101FollowerConfig(
        port=args.port, id=args.robot_id,
        cameras={"top": RealSenseCameraConfig(
            serial_number_or_name=args.camera_serial, fps=int(args.fps),
            width=args.width, height=args.height, color_mode=ColorMode.RGB)},
    )
    robot = SO101FollowerArm(cfg_robot, arm=args.arm)
    harden_bus(robot.bus)
    robot.connect(calibrate=False)
    if not robot.is_calibrated:
        print(f"\n  {args.arm} arm is not calibrated as so101_follower/{args.robot_id}.\n",
              file=sys.stderr)
        robot.disconnect()
        return 1

    # Features come from the robot, not a dataset -- same construction
    # record_kinesthetic.py used, so the layout the policy saw in training is the
    # layout it sees here.
    features = combine_feature_dicts(
        hw_to_dataset_features(robot.action_features, "action", True),
        hw_to_dataset_features(robot.observation_features, "observation", True),
    )

    if not args.no_envelope:
        apply_envelope(robot)
        print(f"  envelope: tau {ARM_TORQUE_LIMIT}, accel {ARM_ACCELERATION}, "
              f"vmax {ARM_MAX_VELOCITY}")
    else:
        print("  envelope NOT applied (--no-envelope)")

    aim_offset: dict[str, float] = {}
    if args.aim_offset:
        for pair in args.aim_offset.split(","):
            name, sep, val = pair.partition("=")
            if not sep:
                raise SystemExit(f"--aim-offset wants JOINT=UNITS, got {pair!r}")
            try:
                aim_offset[f"{name.strip()}.pos"] = float(val)
            except ValueError:
                raise SystemExit(f"--aim-offset: {val!r} is not a number") from None
        known = set(robot.action_features)
        for k in aim_offset:
            if k not in known:
                raise SystemExit(f"--aim-offset: {k!r} is not a joint of this arm "
                                 f"({', '.join(sorted(known))})")
        print("  aim offset: " + ", ".join(f"{k[:-4]} {v:+g}" for k, v in aim_offset.items())
              + "   -- RECORD THIS ALONGSIDE ANY SUCCESS RATE")

    if not 0.0 < args.smooth <= 1.0:
        raise SystemExit("--smooth must be in (0, 1]")
    if args.smooth < 1.0:
        print(f"  smoothing: alpha {args.smooth:g} on body joints (gripper unfiltered)")

    pre.reset()
    post.reset()
    policy.reset()

    frames_dir = None
    if args.log_frames:
        import cv2
        import numpy as np
        args.log_frames.mkdir(parents=True, exist_ok=True)
        frames_dir = args.log_frames
        print(f"  logging a frame every 30 steps to {frames_dir}")

    overlay = None
    if args.overlay:
        import cv2
        import numpy as np
        y0, y1, x0, x1 = (int(v) for v in args.overlay_region.split(","))
        src = cv2.cvtColor(cv2.imread(str(args.overlay)), cv2.COLOR_BGR2RGB)
        overlay = ((slice(y0, y1), slice(x0, x1)), src[y0:y1, x0:x1].copy())
        print(f"  overlay: {args.overlay} region y{y0}:{y1} x{x0}:{x1}"
              "   -- RECORD THIS ALONGSIDE ANY SUCCESS RATE")

    traj_f = traj_w = None
    if args.log_trajectory:
        import csv
        joints = [k[:-4] for k in sorted(robot.action_features)]
        traj_f = open(args.log_trajectory, "w", newline="")
        traj_w = csv.writer(traj_f)
        traj_w.writerow(["step", "t_s"] + [f"state.{j}" for j in joints]
                        + [f"cmd.{j}" for j in joints])
        print(f"  logging trajectory to {args.log_trajectory}")

    infer_ms, loop_ms = [], []
    period = 1.0 / args.fps
    step = 0
    stopped = ""
    peak_c = 0
    prev_cmd = None
    print(f"\n  running {'%d steps' % args.steps if args.steps else '%.0f s' % args.duration}"
          f" at {args.fps:g} Hz target -- Ctrl-C to stop\n")
    t_start = time.perf_counter()

    try:
        while True:
            if args.steps is not None and step >= args.steps:
                break
            if args.steps is None and time.perf_counter() - t_start >= args.duration:
                break
            loop_t = time.perf_counter()

            obs = robot.get_observation()
            if overlay is not None and "top" in obs:
                img = np.array(obs["top"], copy=True)
                img[overlay[0]] = overlay[1]
                obs["top"] = img
            if frames_dir is not None and step % 30 == 0 and "top" in obs:
                cv2.imwrite(str(frames_dir / f"{step:05d}.jpg"),
                            cv2.cvtColor(np.asarray(obs["top"]), cv2.COLOR_RGB2BGR))
            obs_frame = build_dataset_frame(features, obs, prefix=OBS_STR)

            t_inf = time.perf_counter()
            action_values = predict_action(
                observation=obs_frame, policy=policy, device=device,
                preprocessor=pre, postprocessor=post,
                use_amp=cfg.use_amp, task=args.task,
                robot_type=getattr(robot, "robot_type", robot.name),
            )
            infer_ms.append((time.perf_counter() - t_inf) * 1000.0)

            action = make_robot_action(action_values, features)
            for k, dv in aim_offset.items():
                # Applied before smoothing so prev_cmd holds what was actually sent.
                # For a constant offset the filter output is the same either way; this
                # only matters if a future offset is ever made time-varying.
                if k in action:
                    action[k] = float(action[k]) + dv
            if args.smooth < 1.0:
                if prev_cmd is None:
                    # Seed from where the arm actually is. Smoothing against nothing
                    # would send the raw first command, and the opening command is
                    # exactly where a policy started outside its training
                    # distribution lurches hardest.
                    prev_cmd = {k: float(obs[k]) for k in action if k in obs}
                for k in action:
                    # The gripper is deliberately unfiltered. A grasp has to close
                    # decisively; lagging the jaws behind the command is how the arm
                    # arrives at the bottle with the fingers still opening.
                    if k.startswith(GRIPPER_PREFIX) or k not in prev_cmd:
                        continue
                    action[k] = args.smooth * float(action[k]) + (1.0 - args.smooth) * prev_cmd[k]
                prev_cmd = {k: float(v) for k, v in action.items()}
            robot.send_action(action)
            if traj_w is not None:
                # After send_action, so this records the command as issued -- offset
                # and smoothing included. Clamping happens below this layer, in
                # _unnormalize, and is not visible here; compare cmd against the next
                # step's state to see it.
                traj_w.writerow([step, round(time.perf_counter() - t_start, 4)]
                                + [round(float(obs[f"{j}.pos"]), 3) for j in joints]
                                + [round(float(action[f"{j}.pos"]), 3) for j in joints])
            step += 1
            loop_ms.append((time.perf_counter() - loop_t) * 1000.0)

            if step % TEMP_CHECK_EVERY == 0:
                try:
                    temps = robot.bus.sync_read("Present_Temperature", normalize=False)
                    hot = max(int(v) for v in temps.values())
                    if hot >= SERVO_TEMP_CEILING_C:
                        # The bus occasionally returns a corrupted status packet. One
                        # such read said 83 C on a 35 C arm and ended a trial; a servo
                        # cannot gain 30 C in a second, so a ceiling reading only
                        # counts when an immediate re-read agrees with it.
                        again = robot.bus.sync_read("Present_Temperature", normalize=False)
                        hot2 = max(int(v) for v in again.values())
                        if abs(hot2 - hot) > 5:
                            hot = min(hot, hot2)
                    peak_c = max(peak_c, hot)
                    if peak_c >= SERVO_TEMP_CEILING_C:
                        stopped = f"thermal ceiling: {peak_c} C"
                        break
                except Exception:
                    pass
                el = time.perf_counter() - t_start
                sys.stdout.write(f"\r    {step:5d} steps  {el:6.1f}s  "
                                 f"{step / el:5.2f} Hz realised  peak {peak_c} C   ")
                sys.stdout.flush()

            busy_wait(max(0.0, period - (time.perf_counter() - loop_t)))

    except KeyboardInterrupt:
        stopped = "operator stop"
    finally:
        if traj_f is not None:
            traj_f.close()
            print(f"  trajectory written to {args.log_trajectory}")
        sys.stdout.write("\r" + " " * 72 + "\r")
        elapsed = time.perf_counter() - t_start
        try:
            release_gently(robot)
        except Exception:
            pass
        try:
            robot.disconnect()
        except Exception:
            pass

    med = lambda xs: statistics.median(xs) if xs else float("nan")  # noqa: E731
    print("=" * 58)
    if stopped:
        print(f"  stopped early: {stopped}")
    print(f"  inference   {med(infer_ms):6.1f} ms median   "
          f"({min(infer_ms, default=0):.0f}-{max(infer_ms, default=0):.0f} ms)")
    print(f"  loop        {med(loop_ms):6.1f} ms median   "
          f"({min(loop_ms, default=0):.0f}-{max(loop_ms, default=0):.0f} ms)")
    print(f"  peak joint  {peak_c} C")
    print(f"  realised    {step / elapsed if elapsed else 0:.2f} Hz "
          f"over {elapsed:.1f} s")
    print()
    print(f"  STEPS EXECUTED: {step}      <- policy_trials.py asks for this")
    print("=" * 58)
    return 0


if __name__ == "__main__":
    sys.exit(main())
