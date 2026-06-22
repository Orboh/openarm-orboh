#!/usr/bin/env python3
"""
Sanity check: prove that a given arm actually responds to motion commands.

For a single arm bus (canX), this:
  1. Enables all 8 motors.
  2. Reads current J1 (shoulder) position.
  3. Commands J1 to current+5° with a soft MIT gain (Kp=80, Kd=2.0).
  4. Waits 1.5 s while streaming control + state.
  5. Reads new J1 position.
  6. Reports whether J1 moved at least 2° toward the target.
  7. Returns J1 to original pose, disables.

This is the safest possible motion test — only J1 by 5°, low gains, single arm.
Run with /usr/bin/python3 (NOT conda's python).

Usage:
    /usr/bin/python3 ~/Desktop/OpenArm/scripts/openarm_motion_check.py can0
    /usr/bin/python3 ~/Desktop/OpenArm/scripts/openarm_motion_check.py can2
"""

import sys
import time
import math

import openarm_can as oa

if len(sys.argv) < 2 or len(sys.argv) > 3:
    print("usage: openarm_motion_check.py <canX> [Kp]", file=sys.stderr)
    print("  Kp default 80 (good for leader / lightly loaded)", file=sys.stderr)
    print("  Use 300 to overcome follower static friction at J1", file=sys.stderr)
    sys.exit(1)

CAN_IF = sys.argv[1]
KP_J1 = float(sys.argv[2]) if len(sys.argv) == 3 else 80.0

ARM_TYPES = [oa.MotorType.DM8009, oa.MotorType.DM8009,
             oa.MotorType.DM4340, oa.MotorType.DM4340,
             oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310]
SEND = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
RECV = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

print(f"[motion-check] interface: {CAN_IF}")
arm = oa.OpenArm(CAN_IF, True)
arm.init_arm_motors(ARM_TYPES, SEND, RECV)
arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)
arm.set_callback_mode_all(oa.CallbackMode.STATE)

arm.enable_all()
time.sleep(0.1)

# Read initial state (with a few refresh cycles to populate)
for _ in range(20):
    arm.refresh_all()
    arm.recv_all()
    time.sleep(0.01)

motors = arm.get_arm().get_motors()
q0 = motors[0].get_position()
print(f"[motion-check] initial J1 = {q0:+.4f} rad ({math.degrees(q0):+.2f}°)")

target = q0 + math.radians(5.0)
print(f"[motion-check] commanding J1 → {target:+.4f} rad ({math.degrees(target):+.2f}°)  (Kp={KP_J1}, Kd=2.0)")

# Soft hold on the other joints at current pos; move J1 to target
hold_params = []
qs_init = [m.get_position() for m in motors]
for i, q in enumerate(qs_init):
    if i == 0:
        hold_params.append(oa.MITParam(KP_J1, 2.0, target, 0.0, 0.0))
    else:
        kp = 150.0 if i < 4 else 40.0   # follower-safe hold gains
        hold_params.append(oa.MITParam(kp, 2.0, q, 0.0, 0.0))

# Stream control + state for 1.5 s
t_end = time.time() + 1.5
while time.time() < t_end:
    arm.get_arm().mit_control_all(hold_params)
    arm.recv_all()
    time.sleep(0.01)

# Final position read
arm.refresh_all()
arm.recv_all()
q1 = arm.get_arm().get_motors()[0].get_position()
delta = q1 - q0
print(f"[motion-check] final   J1 = {q1:+.4f} rad ({math.degrees(q1):+.2f}°)")
print(f"[motion-check] delta      = {delta:+.4f} rad ({math.degrees(delta):+.2f}°)")

moved_ok = math.degrees(delta) > 2.0
print()
if moved_ok:
    print(f"[OK] J1 moved {math.degrees(delta):+.2f}° → motors respond to commands.")
else:
    print(f"[WARN] J1 moved only {math.degrees(delta):+.2f}° — motors may not be"
          " in MIT control mode, or arm is mechanically blocked.")

# Return to origin softly
return_params = list(hold_params)
return_params[0] = oa.MITParam(KP_J1, 2.0, q0, 0.0, 0.0)
t_end = time.time() + 1.0
while time.time() < t_end:
    arm.get_arm().mit_control_all(return_params)
    arm.recv_all()
    time.sleep(0.01)

arm.disable_all()
print("[motion-check] disabled.")
