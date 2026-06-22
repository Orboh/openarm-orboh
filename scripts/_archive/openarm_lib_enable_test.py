#!/usr/bin/env python3
"""
Diagnostic: does openarm_can library's enable_all() actually green-LED all
8 motors on a given CAN bus, or only a subset?

Sequence:
  1. init_arm_motors (J1..J7)
  2. init_gripper_motor (J8)
  3. set_callback_mode_all(STATE)
  4. enable_all() — watch all 8 LEDs go green if healthy
  5. hold 5 seconds (long enough to count)
  6. disable_all()

Usage:
    /usr/bin/python3 ~/Desktop/OpenArm/scripts/openarm_lib_enable_test.py can0
"""

import sys
import time
import openarm_can as oa

if len(sys.argv) != 2:
    print("usage: openarm_lib_enable_test.py <canX>", file=sys.stderr)
    sys.exit(1)

CAN_IF = sys.argv[1]

print(f"[test] interface: {CAN_IF}")
arm = oa.OpenArm(CAN_IF, True)
print("[test] init_arm_motors ...")
arm.init_arm_motors(
    [oa.MotorType.DM8009, oa.MotorType.DM8009,
     oa.MotorType.DM4340, oa.MotorType.DM4340,
     oa.MotorType.DM4310, oa.MotorType.DM4310, oa.MotorType.DM4310],
    [0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07],
    [0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17])
print("[test] init_gripper_motor ...")
arm.init_gripper_motor(oa.MotorType.DM4310, 0x08, 0x18)
print("[test] set_callback_mode_all(STATE) ...")
arm.set_callback_mode_all(oa.CallbackMode.STATE)

print()
print("[test] >>> calling enable_all() now — WATCH LED COLORS <<<")
arm.enable_all()
print("[test] holding 5 seconds. Count how many motors are GREEN:")
for s in range(5, 0, -1):
    print(f"       {s} ...")
    time.sleep(1)

print("[test] disable_all()")
arm.disable_all()
time.sleep(0.2)
print("[test] done")
