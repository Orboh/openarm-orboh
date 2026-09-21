#!/usr/bin/env python
"""Damiao モータのエラーコードを CAN 受信だけで読む（passive・teleop 稼働中でも安全）。

Damiao のフィードバックフレーム（motor_config の master ID 0x11..0x18 = joint_1..7, gripper）は
byte0 の上位 4bit が状態/エラー: 8=過電圧 9=低電圧 A=過電流 B=MOS過熱 C=モータコイル(巻線)過温
D=通信断 E=過負荷（DM-J4310-2EC V1.1 manual p.6/p.9 https://files.seeedstudio.com/products/Damiao/DM-J4310-en.pdf ）。
0=disable 1=enable は公式表に記載なし・実機観測による（2026-09-21: teleop 中は 1、0xFB クリア後/電源再投入後は 0）。
byte6=MOS平均温度[C]、byte7=コイル平均温度[C]（マニュアル表記は T_Rotor だが本文の定義は「モータ内部コイルの
平均温度」。測定値か推定値かはマニュアル非公開）。openarm_can（dm_motor_control.cpp の parse_motor_state_data）は
byte0 を一切デコードしない（StateResult に状態フィールドが無い）ので、この状態コード表の照合先はマニュアルのみ。
フィードバックはホスト側（teleop 等）が指令/refresh を送ったときだけ返るので、何も動いていない
バスでは受信 0 件になる（その場合は openarm_motorcheck.py を使う）。

使い方:
  ~/projects/lerobot/.venv/bin/python scripts/damiao_errcheck.py can_fol_r [can_lead_r ...] [--seconds 2]
  ... damiao_errcheck.py can_fol_r can_fol_l --clear                 # エラーのモータへクリア(0xFB)を送る（teleop 停止中）
  ... damiao_errcheck.py x --clear-motor can_fol_r:joint_7            # 明示クリア（受信待ちなし）

出力の Trotor はコイル(巻線)の平均温度[C]（マニュアル表記 T_Rotor。OT_Value=100C が過温保護のしきい値）。
"""
from __future__ import annotations

import argparse
import sys
import threading
import time

# 0xC: マニュアル表記は「コイル(巻線)過温」（旧名 ROTOR_OVERTEMP。他スクリプトからの参照なし、2026-09-21 grep 確認済み）
STATE = {0: "disabled", 1: "enabled", 8: "OVER_VOLT", 9: "UNDER_VOLT", 0xA: "OVER_CURRENT", 0xB: "MOS_OVERTEMP",
         0xC: "COIL_OVERTEMP", 0xD: "COMM_LOSS", 0xE: "OVERLOAD"}
NAMES = {0x11: "joint_1", 0x12: "joint_2", 0x13: "joint_3", 0x14: "joint_4", 0x15: "joint_5", 0x16: "joint_6",
         0x17: "joint_7", 0x18: "gripper"}


def sniff(iface: str, seconds: float) -> dict:
    import can

    bus = can.Bus(channel=iface, interface="socketcan", fd=True)
    last: dict[int, tuple] = {}
    worst: dict[int, tuple] = {}   # 観測中に見えたエラー状態（コード, 回数, 初出時刻[s]）
    gaps: dict[int, float] = {}    # 観測中の最大フレーム間隔[ms]（応答が途切れた長さ）
    peak: dict[int, tuple] = {}    # 観測中の最高温度 (Tmos, Trotor)。ラッチ後に読む温度は既に冷えている可能性があるので
    # （2026-09-21: OT_Value=100C に対し latched 状態で Trotor=42C を観測。ラッチ瞬間の値ではない）、観測中のピークも残す
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
    2026-09-21 実機: COIL_OVERTEMP(0xC) のラッチが電源再投入なしで disabled に戻った（右 J7・左 gripper）。
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


# RID: 名前 -> (レジスタ番号, "u32" か "f32")。dm_motor_control.cpp L104-121, L238-241 準拠。
# u32 の RID 帯は 7-10, 13-16, 35-36（それ以外は float32）。しきい値/リミット系のみ抜粋。
PARAM_RIDS = {
    "UV_Value": (0, "f32"), "OT_Value": (2, "f32"), "OC_Value": (3, "f32"),
    "hw_ver": (13, "u32"), "sw_ver": (14, "u32"),
    "PMAX": (21, "f32"), "VMAX": (22, "f32"), "TMAX": (23, "f32"),
    "OV_Value": (29, "f32"), "sub_ver": (36, "u32"),
}


def read_params(iface: str, mid: int, rids: dict) -> dict:
    """RID 読み出し（0x7FF に [id_lo,id_hi,0x33,RID,0,0,0,0] を送り、master ID から
    [id_lo,id_hi,0x33 or 0x55,RID,4B] の応答を受ける。teleop 稼働中は送らない（指令と交錯する）。"""
    import struct

    import can

    sid = mid - 0x10
    id_lo, id_hi = sid & 0xFF, (sid >> 8) & 0xFF
    bus = can.Bus(channel=iface, interface="socketcan", fd=True)
    out = {}
    try:
        for name, (rid, kind) in rids.items():
            bus.send(can.Message(arbitration_id=0x7FF, data=bytes([id_lo, id_hi, 0x33, rid, 0, 0, 0, 0]),
                                  is_extended_id=False, is_fd=False))
            t_end = time.monotonic() + 0.3
            val = None
            while time.monotonic() < t_end:
                m = bus.recv(timeout=0.1)
                if m and m.arbitration_id == mid and len(m.data) >= 8 and m.data[2] in (0x33, 0x55) and m.data[3] == rid:
                    raw = bytes(m.data[4:8])
                    val = struct.unpack("<I", raw)[0] if kind == "u32" else struct.unpack("<f", raw)[0]
                    break
            out[name] = val
    finally:
        bus.shutdown()
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ifaces", nargs="+")
    ap.add_argument("--seconds", type=float, default=2.0)
    ap.add_argument("--clear", action="store_true", help="エラー状態のモータにクリア(0xFB)を送って解除を試みる（teleop 停止中に）")
    ap.add_argument("--clear-motor", action="append", default=[], help="iface:joint_name を明示してクリア（例 can_fol_r:joint_7）")
    ap.add_argument("--params", action="store_true",
                     help="各 iface の全モータの UV/OT/OC/OV/PMAX/VMAX/TMAX/hw_ver/sw_ver/sub_ver を読む（teleop 停止中のみ）")
    args = ap.parse_args()

    if args.params:
        for iface in args.ifaces:
            print(f"== {iface} params (teleop 停止中のみ有効な値)")
            for mid in sorted(NAMES):
                vals = read_params(iface, mid, PARAM_RIDS)
                line = " ".join(f"{k}={v}" for k, v in vals.items())
                print(f"   {NAMES[mid]:8s} {line}")
        return 0

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
            flag = "" if st in (0, 1) else "  <-- ERROR LATCH（teleop 停止後に --clear で解除可。0x9/0xD は原因除去が先）"
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
