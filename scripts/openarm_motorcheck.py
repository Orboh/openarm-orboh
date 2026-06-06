#!/usr/bin/env python3
"""
Motor health + current-angle reporter for all 4 OpenArm CAN buses.

For each arm in the can_map.env, briefly enable -> read current position ->
disable, and print a per-joint table.

Use cases:
 * Verify CAN comm with every one of the 32 motors (factory or post-power-up).
 * Decide whether zero-position calibration is needed: with all 4 arms held
   physically in the "lowered straight down" pose, every J1..J7 reading should
   be close to 0 rad (within ~0.05 rad). If a joint is far off, that arm needs
   zero-calibration.

Run with SYSTEM python so the openarm_can module from python3-openarm-can
(cpython-310) is importable, NOT conda's python:

    /usr/bin/python3 /home/kota-ueda/Desktop/OpenArm/scripts/openarm_motorcheck.py
"""

import os
import time
import argparse
import sys

import openarm_can as oa

ARM_MOTOR_TYPES = [
    oa.MotorType.DM8009,  # J1
    oa.MotorType.DM8009,  # J2
    oa.MotorType.DM4340,  # J3
    oa.MotorType.DM4340,  # J4
    oa.MotorType.DM4310,  # J5
    oa.MotorType.DM4310,  # J6
    oa.MotorType.DM4310,  # J7
]
ARM_SEND_IDS = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
ARM_RECV_IDS = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]
GRIPPER_TYPE, GRIPPER_SEND, GRIPPER_RECV = oa.MotorType.DM4310, 0x08, 0x18


def check_arm(can_iface: str, label: str, settle_s: float = 0.4):
    """Init -> enable -> sample positions for 0.4s -> disable. Return list of
    (joint, pos_rad, vel, torque) tuples."""
    print(f"\n────── {label} on {can_iface} ──────")
    arm = oa.OpenArm(can_iface, True)
    arm.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS)
    arm.init_gripper_motor(GRIPPER_TYPE, GRIPPER_SEND, GRIPPER_RECV)
    arm.set_callback_mode_all(oa.CallbackMode.STATE)
    arm.enable_all()
    time.sleep(0.05)
    arm.refresh_all()
    arm.recv_all()

    t_end = time.time() + settle_s
    while time.time() < t_end:
        arm.refresh_all()
        arm.recv_all()
        time.sleep(0.01)

    arm_motors = arm.get_arm().get_motors()
    grip_motors = arm.get_gripper().get_motors()

    rows = []
    for i, m in enumerate(arm_motors, start=1):
        rows.append((f"J{i}", m.get_position(), m.get_velocity(), m.get_torque()))
    for m in grip_motors:
        rows.append(("GRIP", m.get_position(), m.get_velocity(), m.get_torque()))

    arm.disable_all()

    print(f"{'joint':<6}{'pos[rad]':>12}{'pos[deg]':>12}{'vel':>10}{'tau':>10}  ok?")
    import math
    bad = 0
    for j, p, v, t in rows:
        ok = abs(p) < 0.1 and abs(v) < 0.2
        if not ok:
            bad += 1
        print(f"{j:<6}{p:>12.4f}{math.degrees(p):>12.2f}{v:>10.3f}{t:>10.3f}  "
              f"{'✓' if ok else '✗'}")
    if bad == 0:
        print(f"[OK] {label}: all 8 joints near-zero (calibration looks valid).")
    else:
        print(f"[WARN] {label}: {bad}/8 joints not near-zero → may need calibration.")
    return rows


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--env", default=os.path.expanduser("~/.config/openarm/can_map.env"))
    p.add_argument("--only", help="comma-sep subset of LEADER_R,LEADER_L,FOLLOWER_R,FOLLOWER_L")
    args = p.parse_args()

    if not os.path.exists(args.env):
        print(f"[ERROR] env file not found: {args.env}", file=sys.stderr)
        print("Run: sudo ~/Desktop/OpenArm/scripts/openarm_canup.sh", file=sys.stderr)
        sys.exit(1)

    env = {}
    for line in open(args.env):
        line = line.strip()
        if line.startswith("export ") and "=" in line:
            k, v = line[len("export "):].split("=", 1)
            env[k] = v

    role_order = ["LEADER_R", "LEADER_L", "FOLLOWER_R", "FOLLOWER_L"]
    if args.only:
        role_order = [r.strip().upper() for r in args.only.split(",")]

    for role in role_order:
        key = f"OPENARM_{role}"
        iface = env.get(key)
        if not iface:
            print(f"[skip] {key} not found in env")
            continue
        try:
            check_arm(iface, role)
        except Exception as e:
            print(f"[ERROR] {role} on {iface}: {e}")


if __name__ == "__main__":
    main()
