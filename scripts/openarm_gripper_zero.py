#!/usr/bin/env python3
"""
Gripper-only zero re-calibration for a single OpenArm.

Workflow:
  1. Init arm + gripper (no enable yet — all motors stay disabled / red LED).
  2. Wait for user to physically push the gripper to fully-closed and HOLD.
  3. Enable ONLY the gripper motor (other 7 arm motors stay disabled).
  4. Read current gripper position (just for logging).
  5. Send set_zero on the gripper → motor flash now treats this physical pose
     as 0 rad.
  6. Read gripper position again — should now be ~0.000.
  7. Disable gripper. User can release.

Run for each of the 4 arms after fixing the gripper position by hand.
This avoids the buggy `openarm-can-zero-position-calibration` gripper logic
that caused the previous incident.

Usage:
    /usr/bin/python3 ~/Desktop/OpenArm/scripts/openarm_gripper_zero.py can0
    /usr/bin/python3 ~/Desktop/OpenArm/scripts/openarm_gripper_zero.py can1
    /usr/bin/python3 ~/Desktop/OpenArm/scripts/openarm_gripper_zero.py can2
    /usr/bin/python3 ~/Desktop/OpenArm/scripts/openarm_gripper_zero.py can3
"""

import sys
import time
import math

import openarm_can as oa

if len(sys.argv) != 2:
    print("usage: openarm_gripper_zero.py <canX>", file=sys.stderr)
    sys.exit(1)

CAN_IF = sys.argv[1]

ARM_TYPES = [
    oa.MotorType.DM8009, oa.MotorType.DM8009,
    oa.MotorType.DM4340, oa.MotorType.DM4340,
    oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310]
ARM_SEND = [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07]
ARM_RECV = [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17]

print(f"[gripper-zero] interface: {CAN_IF}")
print("[gripper-zero] init OpenArm (motors not yet enabled — all LEDs should be RED)")
arm = oa.OpenArm(CAN_IF, True)
arm.init_arm_motors(ARM_TYPES, ARM_SEND, ARM_RECV)
arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)
arm.set_callback_mode_all(oa.CallbackMode.STATE)

gripper = arm.get_gripper()

print()
print("=" * 60)
print(f"  PHYSICAL ACTION REQUIRED ({CAN_IF})")
print("=" * 60)
print(" 1. Push this arm's gripper FULLY CLOSED with your hands.")
print(" 2. Keep holding it firmly closed (gentle but firm).")
print(" 3. Make sure NO other arm is being touched.")
print(" 4. When ready, press Enter to enable the gripper motor and burn")
print("    the current position as the new flash zero.")
print()
input("    Press Enter when gripper is held closed > ")

print("[gripper-zero] enabling gripper only (other 7 motors stay disabled)")
gripper.enable_all()
time.sleep(0.3)
for _ in range(20):
    arm.refresh_all()
    arm.recv_all()
    time.sleep(0.01)

g_motor = gripper.get_motors()[0]
pos_before = g_motor.get_position()
print(f"[gripper-zero] gripper pos BEFORE set_zero = "
      f"{pos_before:+.4f} rad ({math.degrees(pos_before):+.2f}°)")

print("[gripper-zero] disabling before flash write (required by DM protocol)")
gripper.disable_all()
time.sleep(0.3)

print("[gripper-zero] writing zero to motor flash...")
# set_zero_all on the gripper component only writes the one gripper motor's zero
gripper.set_zero_all()
time.sleep(0.5)

print("[gripper-zero] re-enabling briefly to verify new zero is read back as ~0")
gripper.enable_all()
time.sleep(0.3)
for _ in range(20):
    arm.refresh_all()
    arm.recv_all()
    time.sleep(0.01)

pos_after = gripper.get_motors()[0].get_position()
print(f"[gripper-zero] gripper pos AFTER  set_zero = "
      f"{pos_after:+.4f} rad ({math.degrees(pos_after):+.2f}°)")

if abs(pos_after) < 0.05:
    print(f"[OK] Gripper on {CAN_IF} zeroed at closed pose "
          f"({math.degrees(pos_after):+.2f}°). You may release.")
else:
    print(f"[WARN] Gripper post-zero pos = {math.degrees(pos_after):+.2f}° "
          f"(expected near 0). Re-run if you weren't fully holding closed.")

gripper.disable_all()
print("[gripper-zero] gripper disabled. Release the gripper now.")
