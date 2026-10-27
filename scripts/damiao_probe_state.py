#!/usr/bin/env python3
"""Damiao モータの状態/エラーコードを enable 1 回で読む（1 モータ・最小侵襲）。

なぜ要るか: Damiao はエラーをレジスタに持たない（RID 一覧は設定値と物理定数のみ）。
状態/エラーはフィードバック 8 バイトの D[0] 上位 4bit にだけ出て、そのフレームは
ホストが指令を送ったときだけ返る。disable 状態のモータは 0x0 しか返さないので、
「LED は赤点滅なのに damiao_errcheck.py は state=disabled」という食い違いが起きる。
本スクリプトは enable(0xFC) を 1 回だけ送って応答の D[0] を読み、直後に
disable(0xFD) で戻す。位置指令（MIT フレーム）は一切送らないので、enable しても
出力トルクは 0 のまま（kp=kd=tau=0 相当）。

使い方（teleop / record / rollout が止まっていること）:
  python scripts/damiao_probe_state.py can_fol_r gripper
  python scripts/damiao_probe_state.py can_fol_r joint_7 --repeat 3

出力の state が 0x1（enabled）なら異常なし。0x8〜0xE ならその種別が確定し、
damiao_errcheck.py --clear-motor <iface>:<joint> で解除を試みる。

2026-09-23 追加（rollout 後に右 follower gripper だけ LED が赤点滅し、passive の
errcheck では state=disabled としか読めなかったため）。
"""
from __future__ import annotations

import argparse
import sys
import time

SEND_IDS = {"joint_1": 0x01, "joint_2": 0x02, "joint_3": 0x03, "joint_4": 0x04,
            "joint_5": 0x05, "joint_6": 0x06, "joint_7": 0x07, "gripper": 0x08}
STATE = {0x0: "disabled", 0x1: "enabled", 0x8: "OVERVOLTAGE", 0x9: "UNDERVOLTAGE",
         0xA: "OVERCURRENT", 0xB: "MOS_OVERTEMP", 0xC: "COIL_OVERTEMP",
         0xD: "COMM_LOSS", 0xE: "OVERLOAD"}


def probe(iface: str, joint: str, repeat: int, settle_s: float) -> int:
    import can

    sid = SEND_IDS[joint]
    mid = sid + 0x10
    bus = can.Bus(channel=iface, interface="socketcan", fd=True)
    seen: list[tuple[int, int, int]] = []
    try:
        for i in range(repeat):
            bus.send(can.Message(arbitration_id=sid, data=bytes([0xFF] * 7 + [0xFC]),
                                 is_extended_id=False, is_fd=False))
            t_end = time.monotonic() + settle_s
            while time.monotonic() < t_end:
                m = bus.recv(timeout=0.05)
                if m and m.arbitration_id == mid and len(m.data) >= 8:
                    seen.append((m.data[0] >> 4, m.data[6], m.data[7]))
                    break
            bus.send(can.Message(arbitration_id=sid, data=bytes([0xFF] * 7 + [0xFD]),
                                 is_extended_id=False, is_fd=False))
            time.sleep(0.05)
    finally:
        # 念のため最後にもう一度 disable を送って必ず脱力側で終える
        try:
            bus.send(can.Message(arbitration_id=sid, data=bytes([0xFF] * 7 + [0xFD]),
                                 is_extended_id=False, is_fd=False))
        finally:
            bus.shutdown()

    if not seen:
        print(f"{iface} {joint}: 応答なし（モータ未給電 / CAN 断 / ID 違い）")
        return 2
    for n, (st, tmos, trotor) in enumerate(seen, 1):
        label = STATE.get(st, f"unknown(0x{st:X})")
        flag = "" if st in (0x0, 0x1) else "   <-- ERROR LATCH"
        print(f"{iface} {joint} [{n}]: state=0x{st:X} {label:14s} Tmos={tmos:3d}C Trotor={trotor:3d}C{flag}")
    worst = max(st for st, _, _ in seen)
    if worst >= 0x8:
        print(f"\n解除を試すなら: python scripts/damiao_errcheck.py {iface} --clear-motor {iface}:{joint}")
        print("解除できなければアームの電源を OFF → 10 秒 → ON（power cycle）")
        return 1
    print("\nエラーラッチは無い（enable が通った）。LED の赤点滅は電気的な異常を示していない可能性が高い。")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("iface", help="can_fol_r / can_fol_l / can_lead_r / can_lead_l")
    ap.add_argument("joint", choices=sorted(SEND_IDS), help="調べる関節（gripper = J8）")
    ap.add_argument("--repeat", type=int, default=2, help="enable→読み→disable を何回繰り返すか（既定 2）")
    ap.add_argument("--settle", type=float, default=0.3, help="応答待ち [s]（既定 0.3）")
    return probe(args.iface, args.joint, args.repeat, args.settle) if (args := ap.parse_args()) else 2


if __name__ == "__main__":
    sys.exit(main())
