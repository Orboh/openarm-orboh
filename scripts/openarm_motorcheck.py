#!/usr/bin/env python3
"""OpenArm 全アームのモータ health＋零位較正チェック（固定名・channel_id 運用版）

各アームを briefly enable → 現在位置を読む → disable し、関節ごとの表を出す。
enable のみ・指令なし＝トルクは入らない（MIT）。腕は手で動かせるまま。

⚠ teleop 等が同じバスを使用中は実行しない（モータ制御が競合する）。

用途:
 * CAN 通信確認: 32モータすべてが応答するか（電源投入後・故障切り分け）。
 * 零位較正の要否判定: 4アームを物理的に「まっすぐ下げた」姿勢に保持した状態で、
   各 J1..J7 が 0 rad 付近（~0.05 rad 以内）か。大きくずれていればそのアームは
   零位較正が要る（near-zero を ✓/✗ で表示）。

固定名（canup.sh / udev と同じ `can_lead_r` 等）を直接使う。旧 can_map.env は不要。

実行（openarm_can は apt の python3-openarm-can なのでシステム python で）:
    sudo bash scripts/canup.sh                                   # 先に固定名 CAN を UP
    /usr/bin/python3 scripts/openarm_motorcheck.py               # 全4アーム
    /usr/bin/python3 scripts/openarm_motorcheck.py --only FOLLOWER_R,FOLLOWER_L
    /usr/bin/python3 scripts/openarm_motorcheck.py --only can_lead_r   # IF 名直接も可
"""

import os
import time
import argparse
import sys
import math

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


# 役割 → 固定名（canup.sh / udev と同じ。channel_id+dev_id ベース・USBポート非依存）
ROLE_IFACE = {
    "LEADER_R": "can_lead_r",
    "LEADER_L": "can_lead_l",
    "FOLLOWER_R": "can_fol_r",
    "FOLLOWER_L": "can_fol_l",
}


def iface_ready(iface):
    """固定名 IF が存在し admin UP か（openarm_can で通信する前提）。"""
    base = f"/sys/class/net/{iface}"
    if not os.path.exists(base):
        return False
    try:
        with open(f"{base}/flags") as f:
            return bool(int(f.read().strip(), 16) & 0x1)  # IFF_UP
    except OSError:
        return False


def main():
    p = argparse.ArgumentParser(
        description="OpenArm 全アームのモータ health＋零位較正チェック（固定名）")
    p.add_argument(
        "--only",
        help="カンマ区切りの部分集合。役割（LEADER_R,LEADER_L,FOLLOWER_R,FOLLOWER_L）"
             "または IF 名（can_lead_r 等）。既定は全4アーム")
    args = p.parse_args()

    targets = ["LEADER_R", "LEADER_L", "FOLLOWER_R", "FOLLOWER_L"]
    if args.only:
        targets = [t.strip() for t in args.only.split(",") if t.strip()]

    any_ran = False
    for t in targets:
        key = t.upper()
        if key in ROLE_IFACE:
            iface, label = ROLE_IFACE[key], key
        elif t.startswith("can"):
            iface, label = t, t          # IF 名を直接指定
        else:
            print(f"[skip] 未知の指定: {t}（役割 LEADER_R 等 か IF 名 can_lead_r 等）")
            continue
        if not iface_ready(iface):
            print(f"[skip] {label} ({iface}): IF が不在 or DOWN"
                  " → 先に `sudo bash scripts/canup.sh` を実行")
            continue
        try:
            check_arm(iface, label)
            any_ran = True
        except Exception as e:
            print(f"[ERROR] {label} on {iface}: {e}")

    if not any_ran:
        print("\n[!] 1本も実行できなかった。"
              "`sudo bash scripts/canup.sh` で固定名 CAN を UP してから再実行。",
              file=sys.stderr)
        sys.exit(1)


if __name__ == "__main__":
    main()
