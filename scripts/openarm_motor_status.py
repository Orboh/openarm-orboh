#!/usr/bin/env python3
"""OpenArm モータ状態ライブモニタ（度数表示・読み取り専用）

rad 表示は直感的に読みづらいため、関節角を度数で表示する可読性向上ツール。
公式 openarm-can-motor-sampling-check の代替（表示のみ・解析機能なし）。

- enable → 状態読み取り → 終了時 disable（公式 sampling-check と同方式）。
  MIT モードでは enable のみ・指令なし＝トルクは入らない（腕は手で動かせるまま）
- ⚠ teleop 等が同じバスを使用中は実行しない（モータ制御が競合する）
- 関節名ラベル付き（J1 肩パン 〜 J7 手首回転＋グリッパ）・温度は MOS 温度

Usage（openarm_can は apt の python3-openarm-can なのでシステム python で実行）:
  /usr/bin/python3 scripts/openarm_motor_status.py can_fol_l            # 2Hz ライブ
  /usr/bin/python3 scripts/openarm_motor_status.py can_lead_l --hz 5
  /usr/bin/python3 scripts/openarm_motor_status.py can_fol_r --once     # 1回だけ
"""

import argparse
import math
import sys
import time
import unicodedata

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

# (短縮名, 説明, モータ型) — 土台側から順
LABELS = [
    ("J1", "肩パン",     "DM8009"),
    ("J2", "肩リフト",   "DM8009"),
    ("J3", "上腕ロール", "DM4340"),
    ("J4", "肘",         "DM4340"),
    ("J5", "前腕ロール", "DM4310"),
    ("J6", "手首ピッチ", "DM4310"),
    ("J7", "手首回転",   "DM4310"),
    ("GR", "グリッパ",   "DM4310"),
]


def sample(arm, duration=0.25):
    """refresh→recv を duration 秒回して最新状態を取り込む。
    注意: refresh 直後に recv する高速ループでないと状態が反映されない（実測。
    openarm_motorcheck.py と同パターン）。"""
    t_end = time.time() + duration
    while time.time() < t_end:
        arm.refresh_all()
        arm.recv_all()
        time.sleep(0.01)


def _w(s):
    """表示幅（全角=2）。日本語ラベル混在でも列が揃うように。"""
    return sum(2 if unicodedata.east_asian_width(c) in "WF" else 1 for c in s)


def _pad(s, width):
    return s + " " * max(0, width - _w(s))


def render(arm, iface, live, frame=0):
    motors = list(arm.get_arm().get_motors()) + list(arm.get_gripper().get_motors())
    lines = []
    lines.append(f"OpenArm motor status  [{iface}]  {time.strftime('%H:%M:%S')}"
                 + (f"  frame={frame}" if live else ""))
    lines.append("-" * 64)
    lines.append(_pad("joint", 6) + _pad("name", 12)
                 + f"{'pos[deg]':>9} {'pos[rad]':>9} {'vel[d/s]':>9} {'tau[Nm]':>8} {'T[C]':>5}")
    for (short, name, _mtype), m in zip(LABELS, motors):
        p, v = m.get_position(), m.get_velocity()
        lines.append(_pad(short, 6) + _pad(name, 12)
                     + f"{math.degrees(p):>+9.1f} {p:>9.4f} {math.degrees(v):>9.1f}"
                     + f" {m.get_torque():>8.3f} {m.get_state_tmos():>5.0f}")
    lines.append("-" * 64)
    if live:
        lines.append("(Ctrl-C で終了)")
        print("\033[2J\033[H" + "\n".join(lines), flush=True)
    else:
        print("\n".join(lines))


def main():
    p = argparse.ArgumentParser(description="OpenArm モータ状態モニタ（度数・読み取り専用）")
    p.add_argument("iface", help="CAN interface（例: can_fol_l / can_lead_r）")
    p.add_argument("--hz", type=float, default=2.0, help="更新頻度 [Hz]（default: 2）")
    p.add_argument("--once", action="store_true", help="1回表示して終了")
    args = p.parse_args()

    try:
        arm = oa.OpenArm(args.iface, True)
    except Exception as e:
        print(f"[ERROR] {args.iface} を開けない: {e}", file=sys.stderr)
        print("CAN が UP か確認: sudo bash scripts/canup.sh", file=sys.stderr)
        sys.exit(1)

    arm.init_arm_motors(ARM_MOTOR_TYPES, ARM_SEND_IDS, ARM_RECV_IDS)
    arm.init_gripper_motor(GRIPPER_TYPE, GRIPPER_SEND, GRIPPER_RECV)
    arm.set_callback_mode_all(oa.CallbackMode.STATE)
    # enable しないと refresh に状態が返らない（実測）。enable のみ・指令なし＝トルク無し。
    arm.enable_all()
    time.sleep(0.05)
    try:
        sample(arm, duration=0.4)  # 初回 warmup
        if args.once:
            render(arm, args.iface, live=False)
            return
        frame = 0
        while True:
            sample(arm)
            frame += 1
            render(arm, args.iface, live=True, frame=frame)
            time.sleep(max(0.0, 1.0 / args.hz))
    except KeyboardInterrupt:
        print("\nbye")
    finally:
        arm.disable_all()  # 必ず disable して終了


if __name__ == "__main__":
    main()
