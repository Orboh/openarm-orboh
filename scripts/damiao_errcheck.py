#!/usr/bin/env python
"""Damiao モータのエラーコードを CAN 受信だけで読む（passive・teleop 稼働中でも安全）。

Damiao のフィードバックフレーム（motor_config の master ID 0x11..0x18 = joint_1..7, gripper）は
byte0 の上位 4bit が状態/エラー: 0=disable 1=enable 8=過電圧 9=低電圧 A=過電流 B=MOS過熱
C=ロータ過熱 D=通信断 E=過負荷。byte6=MOS温度[C] byte7=ロータ温度[C]。
フィードバックはホスト側（teleop 等）が指令/refresh を送ったときだけ返るので、何も動いていない
バスでは受信 0 件になる（その場合は openarm_motorcheck.py を使う）。

使い方:
  ~/projects/lerobot/.venv/bin/python scripts/damiao_errcheck.py can_fol_r [can_lead_r ...] [--seconds 2]
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

STATE = {0: "disabled", 1: "enabled", 8: "OVER_VOLT", 9: "UNDER_VOLT", 0xA: "OVER_CURRENT", 0xB: "MOS_OVERTEMP",
         0xC: "ROTOR_OVERTEMP", 0xD: "COMM_LOSS", 0xE: "OVERLOAD"}
NAMES = {0x11: "joint_1", 0x12: "joint_2", 0x13: "joint_3", 0x14: "joint_4", 0x15: "joint_5", 0x16: "joint_6",
         0x17: "joint_7", 0x18: "gripper"}


def sniff(iface: str, seconds: float) -> dict:
    import can

    bus = can.Bus(channel=iface, interface="socketcan", fd=True)
    last: dict[int, tuple] = {}
    count = 0
    t_end = time.monotonic() + seconds
    try:
        while time.monotonic() < t_end:
            msg = bus.recv(timeout=0.2)
            if msg is None or msg.arbitration_id not in NAMES or len(msg.data) < 8:
                continue
            count += 1
            b = msg.data
            last[msg.arbitration_id] = (b[0] >> 4, b[6], b[7])
    finally:
        bus.shutdown()
    return {"count": count, "last": last}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ifaces", nargs="+")
    ap.add_argument("--seconds", type=float, default=2.0)
    args = ap.parse_args()
    results = {}
    threads = [threading.Thread(target=lambda i=i: results.__setitem__(i, sniff(i, args.seconds))) for i in args.ifaces]
    for t in threads:
        t.start()
    for t in threads:
        t.join()
    rc = 0
    for iface in args.ifaces:
        r = results.get(iface)
        if r is None:
            print(f"== {iface}: open failed")
            rc = 1
            continue
        print(f"== {iface}: {r['count']} feedback frames in {args.seconds:g}s")
        if r["count"] == 0:
            print("   (受信なし: ホストが指令を送っていないか、モータ未給電。teleop 停止中なら openarm_motorcheck.py で)")
            continue
        for mid in sorted(NAMES):
            if mid not in r["last"]:
                print(f"   {NAMES[mid]:8s} --- no frame ---")
                rc = 1
                continue
            st, tm, tr = r["last"][mid]
            label = STATE.get(st, f"code_{st:X}")
            flag = "" if st in (0, 1) else "  <-- ERROR LATCH（電源再投入でのみ解除）"
            print(f"   {NAMES[mid]:8s} state={label:14s} Tmos={tm:3d}C Trotor={tr:3d}C{flag}")
            if st not in (0, 1):
                rc = 1
    return rc


if __name__ == "__main__":
    sys.exit(main())
