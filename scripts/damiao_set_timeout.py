#!/usr/bin/env python3
"""Damiao モータの通信断タイムアウト（レジスタ TIMEOUT・RID 9・ms・0=無効）を RAM に書く／読む。

背景（2026-09-21 板橋）: leader の重力補償は各関節へ (kp=0, kd=0, tau=G(q)) のトルクだけを送る。
ハーネスの接点が開いて指令が届かなくなった関節は「最後のトルク」を保持して回り続け（TIMEOUT=0 のため
自分では disable にならない）、lerobot の Ctrl-C（disable 1 回送信）も届かない。TIMEOUT を 100〜200 ms に
しておけば、指令が途絶えた関節はモータ自身が disable に落ちる＝ホストが何もできない状況で唯一効く層。

注意:
  - RAM 書き込みのみ（0x55）。電源を切れば 0 に戻る。保存（0xAA）は意図的に使わない。
    → 電源投入のたびに実行する（canup.sh の後、teleop の前）。
  - teleop 稼働中は送らない（0x7FF の指令と交錯する）。既定で 0.3 秒受信して誰かが polling 中なら中止。
  - leader 向け。follower に入れると、ホストが 200 ms 止まった瞬間に腕が脱力して落ちる副作用があるので、
    follower は別途判断する（既定では触らない）。
  - 待機中に指令が止まる場面（整列前後・接続直後など）で脱力する副作用は 1 モータで試してから全体へ。

使い方（Thor・~/projects/openarm-orboh）:
  python scripts/damiao_set_timeout.py can_lead_r --read                 # 現在値を読むだけ
  python scripts/damiao_set_timeout.py can_lead_r --ms 200 --motors 5    # J5 だけ 200 ms
  python scripts/damiao_set_timeout.py can_lead_r --ms 200               # 8 モータ全部
  python scripts/damiao_set_timeout.py can_lead_r --ms 0                 # 無効に戻す（電源再投入でも戻る）

フレーム形式（dm_motor_control.cpp / damiao_errcheck.py と同じ）:
  読み: 0x7FF ← [id_lo, id_hi, 0x33, RID, 0,0,0,0]   応答: master ID(0x10+id) → [id_lo,id_hi,0x33,RID,u32LE]
  書き: 0x7FF ← [id_lo, id_hi, 0x55, RID, u32LE]     応答: master ID → [id_lo,id_hi,0x55,RID,u32LE]
"""

from __future__ import annotations

import argparse
import struct
import sys
import time

RID_TIMEOUT = 9
NAMES = {1: "joint_1", 2: "joint_2", 3: "joint_3", 4: "joint_4", 5: "joint_5", 6: "joint_6", 7: "joint_7", 8: "gripper"}


def _open(iface: str):
    import can

    return can.Bus(channel=iface, interface="socketcan", fd=True)


def _xfer(bus, sid: int, cmd: int, payload: bytes, wait_s: float = 0.3):
    """0x7FF へ 1 フレーム送り、master ID からの同 RID 応答を待つ。返り値は u32 か None。"""
    import can

    mid = 0x10 + sid
    id_lo, id_hi = sid & 0xFF, (sid >> 8) & 0xFF
    bus.send(can.Message(arbitration_id=0x7FF, data=bytes([id_lo, id_hi, cmd, RID_TIMEOUT]) + payload,
                         is_extended_id=False, is_fd=False))
    t_end = time.monotonic() + wait_s
    while time.monotonic() < t_end:
        m = bus.recv(timeout=0.05)
        if (m and m.arbitration_id == mid and len(m.data) >= 8
                and m.data[2] in (0x33, 0x55) and m.data[3] == RID_TIMEOUT):
            return struct.unpack("<I", bytes(m.data[4:8]))[0]
    return None


def bus_busy(bus, seconds: float = 0.3) -> int:
    """誰か（teleop 等）が polling 中かを受信だけで判定。受信フレーム数を返す。"""
    n = 0
    t_end = time.monotonic() + seconds
    while time.monotonic() < t_end:
        if bus.recv(timeout=0.05):
            n += 1
    return n


def main() -> int:
    ap = argparse.ArgumentParser(description="Damiao TIMEOUT(RID 9) を RAM に書く／読む（leader 向け・teleop 停止中に）")
    ap.add_argument("ifaces", nargs="+", help="socketcan インタフェース（例 can_lead_r）")
    ap.add_argument("--ms", type=int, default=None, help="書き込む値 ms（0=無効）。省略時は読むだけ")
    ap.add_argument("--read", action="store_true", help="読むだけ（--ms 無しと同じ）")
    ap.add_argument("--motors", default="1-8", help="slave ID の範囲/列挙（既定 1-8。例 5 / 5-8 / 1,2,5）")
    ap.add_argument("--force", action="store_true", help="バスに通信があっても実行する（通常は使わない）")
    args = ap.parse_args()

    sids: list[int] = []
    for part in args.motors.split(","):
        part = part.strip()
        if "-" in part:
            a, b = part.split("-")
            sids.extend(range(int(a), int(b) + 1))
        elif part:
            sids.append(int(part))
    sids = [s for s in sids if 1 <= s <= 8]
    if not sids:
        print("--motors が空", file=sys.stderr)
        return 2

    write = args.ms is not None and not args.read
    if write and not (0 <= args.ms <= 60000):
        print("--ms は 0〜60000", file=sys.stderr)
        return 2

    rc = 0
    for iface in args.ifaces:
        bus = _open(iface)
        try:
            n = bus_busy(bus)
            if n and not args.force:
                print(f"[{iface}] バスに通信あり（0.3 秒で {n} フレーム）。teleop を止めてから実行する（--force で強行）")
                rc = 1
                continue
            print(f"== {iface}: TIMEOUT(RID 9) {'書き込み ' + str(args.ms) + ' ms' if write else '読み出し'} ==")
            for sid in sids:
                name = NAMES[sid]
                before = _xfer(bus, sid, 0x33, b"\x00\x00\x00\x00")
                if before is None:
                    print(f"   {name:8s} 無応答（未給電か接点開放）")
                    rc = 1
                    continue
                if not write:
                    print(f"   {name:8s} TIMEOUT={before} ms")
                    continue
                w = _xfer(bus, sid, 0x55, struct.pack("<I", args.ms))
                after = _xfer(bus, sid, 0x33, b"\x00\x00\x00\x00")
                ok = after == args.ms
                print(f"   {name:8s} {before} -> {after} ms  {'OK' if ok else 'NG (write resp=' + str(w) + ')'}")
                if not ok:
                    rc = 1
        finally:
            bus.shutdown()
    if write and rc == 0:
        print("RAM のみ（電源を切ると 0 に戻る）。teleop 前に毎回実行すること。")
    return rc


if __name__ == "__main__":
    sys.exit(main())
