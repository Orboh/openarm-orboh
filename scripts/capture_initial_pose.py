#!/usr/bin/env python
"""Capture a teleop start ("ready") pose for one OpenArm side by hand.

Connect to the leader, hand-move it to the desired ready pose (elbow bent, arm
raised, ...) while gravity compensation holds it weightless, and press ENTER.
The measured joint angles (degrees) are written to a per-side YAML that
align_on_connect consumes via ``--...initial_pose_path`` in run_teleop_*.sh /
record_pick_place_openarm.sh.

Start-pose requirement depends on the calibration flow (auto-detected):
- persistent zero (homing-offset calibration, ``scripts/calibrate_openarm.sh``):
  connect() does not re-zero, so the arm can start from ANY pose.
- legacy calibration (offsets all zero): connect() burns the motor zero at the
  current pose, so the arm MUST hang straight down before connecting.

The gripper is intentionally NOT captured -- it is pinned to the calibration
zero (closed) in the saved pose, since commanding a non-zero gripper hold on
startup is what damaged a follower gripper in the field.

Usage (normally via scripts/capture_initial_pose.sh <left|right>):
  python scripts/capture_initial_pose.py --side right --port can_lead_r \
      --id leader_right --gravity-urdf urdf/openarm_v10_bimanual.urdf \
      --gravity-side right --out config/initial_pose_right.yaml
"""

from __future__ import annotations

import argparse
import os
import select
import sys
import time

import yaml

from lerobot.teleoperators.openarm_leader.config_openarm_leader import OpenArmLeaderConfig
from lerobot.teleoperators.openarm_leader.openarm_leader import OpenArmLeader

ARM_JOINTS = [f"joint_{i}" for i in range(1, 8)]


def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(description="Capture an OpenArm teleop start pose by hand.")
    ap.add_argument("--side", required=True, choices=["left", "right"])
    ap.add_argument("--port", required=True, help="leader CAN interface, e.g. can_lead_r")
    ap.add_argument("--id", default=None, help="calibration id, e.g. leader_right (reuses teleop calib)")
    ap.add_argument("--gravity-urdf", default=None, help="dynamics URDF for gravity comp (holds arm weightless)")
    ap.add_argument("--gravity-side", default=None, choices=["left", "right"])
    ap.add_argument("--gravity-scale", type=float, default=1.0)
    ap.add_argument("--no-gravity", action="store_true", help="capture with torque OFF instead of gravity comp")
    ap.add_argument("--out", default=None, help="output YAML path (default config/initial_pose_<side>.yaml)")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    use_gravity = bool(args.gravity_urdf) and not args.no_gravity
    out_path = args.out or os.path.join("config", f"initial_pose_{args.side}.yaml")

    cfg = OpenArmLeaderConfig(
        port=args.port,
        id=args.id or f"leader_{args.side}",
        # Do NOT move on connect -- we are capturing, not aligning.
        align_on_connect=False,
        # Weightless hold while positioning (recommended). Falls back to torque-off.
        gravity_compensation=use_gravity,
        gravity_urdf_path=args.gravity_urdf,
        gravity_side=args.gravity_side or args.side,
        gravity_scale=args.gravity_scale,
        manual_control=not use_gravity,
    )

    leader = OpenArmLeader(cfg)

    print(f"[capture] side={args.side} port={args.port} gravity={'on' if use_gravity else 'off (torque disabled)'}")
    # With a homing-offset calibration (new persistent-zero flow), connect() does
    # NOT re-burn the motor zero, so the start pose is irrelevant. With a legacy
    # calibration (offsets all zero), connect() burns the zero at the current
    # pose and the arm MUST hang straight down for the captured pose to be valid.
    has_offsets = any(cal.homing_offset != 0 for cal in (leader.bus.calibration or {}).values())
    if has_offsets:
        print("[capture] persistent zero active (homing-offset calibration): no re-zero on connect;")
        print("[capture] the arm can start from any pose. Press ENTER to connect.")
    else:
        print("[capture] legacy calibration: HANG THE ARM STRAIGHT DOWN, gripper closed,")
        print("[capture] then press ENTER to connect (burns zero at the current pose).")
    input()

    leader.connect()
    print("[capture] connected. Move the arm to the desired READY pose.")
    print("[capture] Live joint angles below. Press ENTER to capture, Ctrl-C to abort.\n")

    last: dict[str, float] = {}
    next_print = 0.0
    try:
        while True:
            action = leader.get_action()  # keeps gravity injected + reads pos
            last = {k[: -len(".pos")]: v for k, v in action.items() if k.endswith(".pos") and v is not None}

            now = time.perf_counter()
            if now >= next_print:
                shown = "  ".join(f"{j}={last.get(j, float('nan')):6.1f}" for j in ARM_JOINTS)
                sys.stdout.write("\r[capture] " + shown + "   ")
                sys.stdout.flush()
                next_print = now + 0.2

            r, _, _ = select.select([sys.stdin], [], [], 0.0)
            if r:
                sys.stdin.readline()
                break
            time.sleep(0.005)
    finally:
        leader.disconnect()

    print()
    missing = [j for j in ARM_JOINTS if j not in last]
    if missing:
        print(f"[capture] ERROR: no reading for {missing}; nothing written.", file=sys.stderr)
        return 1

    pose = {j: round(float(last[j]), 2) for j in ARM_JOINTS}
    pose["gripper"] = 0.0  # pinned to calibration zero (see module docstring)

    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    with open(out_path, "w") as f:
        f.write(f"# OpenArm teleop start (ready) pose -- side={args.side}, degrees.\n")
        f.write("# Captured by scripts/capture_initial_pose.py. Consumed by align_on_connect\n")
        f.write("# via --...initial_pose_path. gripper pinned to calibration zero.\n")
        yaml.safe_dump(pose, f, sort_keys=False, default_flow_style=False)

    print(f"[capture] wrote {out_path}:")
    for j in ARM_JOINTS + ["gripper"]:
        print(f"    {j}: {pose[j]}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
