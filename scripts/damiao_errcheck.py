#!/usr/bin/env python
"""Damiao モータのエラーコードを CAN 受信だけで読む（passive・teleop 稼働中でも安全）。

Damiao のフィードバックフレーム（motor_config の master ID 0x11..0x18 = joint_1..7, gripper）は
byte0 の上位 4bit が状態/エラー: 0=disable 1=enable 8=過電圧 9=低電圧 A=過電流 B=MOS過熱
C=ロータ過熱 D=通信断 E=過負荷。byte6=MOS温度[C] byte7=ロータ温度[C]。
フィードバックはホスト側（teleop 等）が指令/refresh を送ったときだけ返るので、何も動いていない
バスでは受信 0 件になる（その場合は openarm_motorcheck.py を使う）。

使い方:
  ~/projects/lerobot/.venv/bin/python scripts/damiao_errcheck.py can_fol_r [can_lead_r ...] [--seconds 2]
  ... damiao_errcheck.py can_fol_r can_fol_l --clear                 # エラーのモータへクリア(0xFB)を送る（teleop 停止中）
  ... damiao_errcheck.py x --clear-motor can_fol_r:joint_7            # 明示クリア（受信待ちなし）
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
    worst: dict[int, tuple] = {}   # 観測中に見えたエラー状態（コード, 回数, 初出時刻[s]）
    gaps: dict[int, float] = {}    # 観測中の最大フレーム間隔[ms]（応答が途切れた長さ）
    peak: dict[int, tuple] = {}    # 観測中の最高温度 (Tmos, Trotor)。ラッチは推定コイル温度の瞬間値で起きるので、ピークが証拠になる
    seen_at: dict[int, float] = {}
    count = 0
    t0 = time.monotonic()
    t_end = t0 + seconds
    try:
        while time.monotonic() < t_end:
            msg = bus.recv(timeout=0.2)
            if msg is None or msg.arbitration_id not in NAMES or len(msg.data) < 8:
                continue
            count += 1
            b = msg.data
            mid = msg.arbitration_id
            now = time.monotonic()
            st = b[0] >> 4
            last[mid] = (st, b[6], b[7])
            pk = peak.get(mid, (0, 0))
            peak[mid] = (max(pk[0], b[6]), max(pk[1], b[7]))
            if st not in (0, 1):
                c, n, t, tm0, tr0 = worst.get(mid, (st, 0, now - t0, b[6], b[7]))
                worst[mid] = (st, n + 1, t, tm0, tr0)
            if mid in seen_at:
                gaps[mid] = max(gaps.get(mid, 0.0), (now - seen_at[mid]) * 1000)
            seen_at[mid] = now
    finally:
        bus.shutdown()
    return {"count": count, "last": last, "worst": worst, "gaps": gaps, "peak": peak}


def clear_error(iface: str, mid: int) -> str:
    """Damiao の「エラークリア」(data FF*7 + FB) を slave ID (= master ID - 0x10) に送り、応答の状態を返す。
    2026-09-21 実機: ROTOR_OVERTEMP のラッチが電源再投入なしで disabled に戻った（右 J7・左 gripper）。
    teleop が動いている間は送らない（指令と交錯する）。"""
    import can

    sid = mid - 0x10
    bus = can.Bus(channel=iface, interface="socketcan", fd=True)
    try:
        bus.send(can.Message(arbitration_id=sid, data=bytes([0xFF] * 7 + [0xFB]), is_extended_id=False, is_fd=False))
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            m = bus.recv(timeout=0.2)
            if m and m.arbitration_id == mid and len(m.data) >= 8:
                return STATE.get(m.data[0] >> 4, hex(m.data[0] >> 4))
        return "no reply"
    finally:
        bus.shutdown()


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ifaces", nargs="+")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--clear", action="store_true", help="エラー状態のモータにクリア(0xFB)を送って解除を試みる（teleop 停止中に）")
    ap.add_argument("--clear-motor", action="append", default=[], help="iface:joint_name を明示してクリア（例 can_fol_r:joint_7）")
    args = ap.parse_args()

    if args.clear_motor:
        rc = 0
        for spec in args.clear_motor:
            iface, jn = spec.split(":", 1)
            mid = next((k for k, v in NAMES.items() if v == jn), None)
            if mid is None:
                print(f"unknown joint name: {jn}"); rc = 1; continue
            print(f"clear {iface} {jn}: -> {clear_error(iface, mid)}")
        return rc
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
            w = r["worst"].get(mid)
            seen = f"  seen: {STATE.get(w[0], hex(w[0]))} x{w[1]} (first at {w[2]:.1f}s, then Tmos={w[3]}C Trotor={w[4]}C)" if w else ""
            pk = r["peak"].get(mid, (0, 0))
            gapflag_peak = f"  peak Tmos/Trotor={pk[0]}/{pk[1]}C"
            gap = r["gaps"].get(mid, 0.0)
            gapflag = f"  max-gap={gap:.0f}ms" + ("  <-- 応答途切れ" if gap > 100 else "")
            print(f"   {NAMES[mid]:8s} state={label:14s} Tmos={tm:3d}C Trotor={tr:3d}C{gapflag_peak}{gapflag}{seen}{flag}")
            if st not in (0, 1) or w or gap > 100:
                rc = 1
            if args.clear and st not in (0, 1):
                print(f"            clear -> {clear_error(iface, mid)}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
