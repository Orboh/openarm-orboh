#!/usr/bin/env python
"""4 ユニットの関節角を「生値（native）／logical（lerobot が使う値）／YAML 目標」で並べて毎秒出す受信専用モニター。

なぜ要るか（2026-09-22 板橋）: motorcheck やロガーが出す角度はモータ生値で、lerobot は
生値から較正ファイルの homing_offset を引いた logical を使う（damiao.py: raw = logical + offset）。
leader と follower は offset がまるで違うので、生値どうしを比べても物理の一致は分からない。
整列（位置制御）→ teleop の切り替わりで「leader と follower が揃っているか」「YAML 目標に
着いているか」を logical で見るのが正しい。

受信のみ（送信ゼロ）。teleop / record と並走してよい。teleop が動いていないと生値が来ない
（passive のため）。その場合は `openarm_motorcheck.py` で応答を引き出す。

使い方（Thor・~/projects/openarm-orboh）:
  python scripts/openarm_pose_monitor.py                       # 4 バス・1 秒ごと・Ctrl-C で終了
  python scripts/openarm_pose_monitor.py --once                # 1.5 秒集めて 1 回だけ出す
  python scripts/openarm_pose_monitor.py --interval 0.5 --warn 3
  python scripts/openarm_pose_monitor.py can_lead_l can_fol_l  # 左だけ
  python scripts/openarm_pose_monitor.py --record ~/collections/pose-0922-0930.csv --quiet   # logical を 0.2 秒ごとに CSV へ（teleop の前に起動）
  python scripts/openarm_pose_monitor.py --report ~/collections/pose-0922-0930.csv           # 整列終了・loop 開始・+1/+3 s の姿勢を side ごとに要約

出力の読み方:
  Δ = leader の logical − follower の logical（同じ YAML へ整列した直後は 0 に近いはず）。
  |Δ| が --warn（既定 3°）を超えた行に ★。native の差は無視してよい（offset の差が乗る）。
"""
from __future__ import annotations

import argparse
import json
import math
import os
import signal
import sys
import threading
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from damiao_feedback_log import NAMES, decode  # noqa: E402  (受信フレームのデコードを共用)

REPO = HERE.parent
CALIB_DIR = Path(os.environ.get("CALIB_DIR", Path.home() / ".cache/huggingface/lerobot/calibration"))
UNIT = {
    "can_fol_l": ("left", "follower", CALIB_DIR / "robots/openarm_follower/follower_left.json"),
    "can_fol_r": ("right", "follower", CALIB_DIR / "robots/openarm_follower/follower_right.json"),
    "can_lead_l": ("left", "leader", CALIB_DIR / "teleoperators/openarm_leader/leader_left.json"),
    "can_lead_r": ("right", "leader", CALIB_DIR / "teleoperators/openarm_leader/leader_right.json"),
}
JOINTS = ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6", "joint_7", "gripper"]


def load_offsets(path: Path) -> dict[str, float]:
    try:
        d = json.load(open(path))
    except OSError:
        return {}
    return {k: float(v.get("homing_offset", 0.0)) for k, v in d.items() if isinstance(v, dict)}


def load_yaml_pose(path: Path) -> dict[str, float]:
    out: dict[str, float] = {}
    try:
        for line in open(path, encoding="utf-8"):
            line = line.split("#", 1)[0].strip()
            if ":" in line:
                k, v = line.split(":", 1)
                try:
                    out[k.strip()] = float(v.strip())
                except ValueError:
                    pass
    except OSError:
        pass
    return out


def reader(iface: str, latest: dict, stop: threading.Event) -> None:
    import can

    bus = can.Bus(channel=iface, interface="socketcan", fd=True)
    try:
        while not stop.is_set():
            m = bus.recv(timeout=0.2)
            if m is None or m.arbitration_id not in NAMES or len(m.data) < 8:
                continue
            name = NAMES[m.arbitration_id]
            try:
                d = decode(bytes(m.data), name)
            except Exception:
                continue
            latest[(iface, name)] = (time.monotonic(), math.degrees(d["pos_rad"]))
    finally:
        bus.shutdown()


def run_report(path: Path, warn: float = 3.0) -> int:
    """--record した CSV から、side ごとに「整列終了」「loop 開始」「+1 s」「+3 s」の logical を並べる。

    loop 開始の検出: leader バスは「leader 整列（約 2.2 s）→ follower 接続・整列の空白（約 2.8 s・
    受信なし）→ loop」の順なので、leader の受信が 1 s 以上途切れた後に再開した時刻を loop 開始とする。
    整列終了の値は、leader は空白直前の最後の値、follower は loop 開始直前の値。
    """
    import csv
    from collections import defaultdict

    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    if not rows:
        print("empty"); return 1
    by: dict = defaultdict(list)  # (iface, joint) -> [(t, native, logical, yaml)]
    for r in rows:
        try:
            by[(r["iface"], r["joint"])].append((float(r["t_rel"]), float(r["native_deg"]), float(r["logical_deg"]),
                                                float(r["yaml_deg"]) if r.get("yaml_deg") else None))
        except (KeyError, ValueError):
            continue

    def at(iface: str, j: str, t: float, w: float = 0.35) -> float | None:
        vals = [l for (tt, _, l, _) in by.get((iface, j), []) if t - w <= tt <= t + w]
        return sum(vals) / len(vals) if vals else None

    def last_before(iface: str, j: str, t: float) -> float | None:
        vals = [l for (tt, _, l, _) in by.get((iface, j), []) if tt < t]
        return vals[-1] if vals else None

    for side in ("left", "right"):
        lead = f"can_lead_{side[0]}"; fol = f"can_fol_{side[0]}"
        ts = sorted(t for (i, j), arr in by.items() if i == lead and j == "joint_1" for (t, *_r) in arr)
        if len(ts) < 5:
            print(f"== {side}: leader データ不足（{len(ts)} 行）"); continue
        # gaps in leader stream
        gaps = [(ts[k - 1], ts[k]) for k in range(1, len(ts)) if ts[k] - ts[k - 1] > 1.0]
        if not gaps:
            print(f"== {side}: leader の受信に 1 s 以上の空白が無い＝整列→loop の遷移を含んでいない（連続 {ts[0]:.1f}〜{ts[-1]:.1f} s）")
            t_loop = ts[0]; t_gap0 = None
        else:
            t_gap0, t_loop = gaps[0]
        yaml = {j: next((y for (_, _, _, y) in by.get((lead, j), []) if y is not None), None) for j in JOINTS}
        print(f"== {side}: leader 整列終了≈{t_gap0 if t_gap0 is not None else float('nan'):.1f}s  loop 開始≈{t_loop:.1f}s   （logical [deg]）")
        print(f"   {'joint':8s} {'yaml':>7s} | {'lead@align':>10s} {'fol@align':>9s} {'Δ':>6s} | {'lead+0.1':>8s} {'fol+0.1':>8s} {'Δ':>6s} | {'lead+1':>7s} {'lead+3':>7s} {'fol+3':>7s} | 判定")
        for j in JOINTS:
            la = last_before(lead, j, t_gap0) if t_gap0 is not None else at(lead, j, t_loop)
            fa = last_before(fol, j, t_loop)
            l1 = at(lead, j, t_loop + 0.1); f1 = at(fol, j, t_loop + 0.1)
            l2 = at(lead, j, t_loop + 1.0); l3 = at(lead, j, t_loop + 3.0); f3 = at(fol, j, t_loop + 3.0)
            y = yaml.get(j)
            f = lambda v: "   -   " if v is None else f"{v:7.1f}"
            d_align = (la - fa) if (la is not None and fa is not None) else None
            d_loop = (l1 - f1) if (l1 is not None and f1 is not None) else None
            notes = []
            if y is not None and la is not None and abs(la - y) > warn: notes.append(f"leader 整列で YAML から {la - y:+.1f}")
            if y is not None and fa is not None and abs(fa - y) > warn: notes.append(f"follower 整列で YAML から {fa - y:+.1f}")
            if d_align is not None and abs(d_align) > warn: notes.append("整列時に lead≠fol ★")
            if l1 is not None and l3 is not None and abs(l3 - l1) > 5: notes.append(f"loop 後 leader が {l3 - l1:+.1f} 動く（崩れ）")
            print(f"   {j:8s} {f(y)} | {f(la):>10s} {f(fa):>9s} {f(d_align):>6s} | {f(l1):>8s} {f(f1):>8s} {f(d_loop):>6s} | {f(l2):>7s} {f(l3):>7s} {f(f3):>7s} | {'; '.join(notes)}")
        print()
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ifaces", nargs="*", default=list(UNIT), help="socketcan IF（既定: 4 バス）")
    ap.add_argument("--interval", type=float, default=1.0, help="表示間隔 [s]")
    ap.add_argument("--once", action="store_true", help="1.5 秒集めて 1 回出して終了")
    ap.add_argument("--warn", type=float, default=3.0, help="|Δ| がこの角度 [deg] を超えた行に ★")
    ap.add_argument("--stale", type=float, default=1.0, help="この秒数より古い値は '-' 表示")
    ap.add_argument("--record", metavar="CSV", default=None, help="logical/native を --interval ごとに CSV へ追記（受信のみ）")
    ap.add_argument("--report", metavar="CSV", default=None, help="--record した CSV を要約して終了（送受信しない）")
    ap.add_argument("--quiet", action="store_true", help="表を出さない（--record と併用）")
    args = ap.parse_args()
    if args.report:
        return run_report(Path(args.report), warn=args.warn)
    if args.record and args.interval > 0.5:
        args.interval = 0.2

    offsets = {i: load_offsets(UNIT[i][2]) for i in args.ifaces if i in UNIT}
    yaml_pose = {"left": load_yaml_pose(REPO / "config/initial_pose_left.yaml"),
                 "right": load_yaml_pose(REPO / "config/initial_pose_right.yaml")}
    for i in args.ifaces:
        if i not in UNIT:
            print(f"未知の IF: {i}（{', '.join(UNIT)} のいずれか）", file=sys.stderr)
            return 2
        if not offsets[i]:
            print(f"[warn] 較正ファイルが読めない: {UNIT[i][2]}（logical = native として表示）", file=sys.stderr)

    latest: dict = {}
    stop = threading.Event()
    threads = [threading.Thread(target=reader, args=(i, latest, stop), daemon=True) for i in args.ifaces]
    for t in threads:
        t.start()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    t0 = time.monotonic()
    rec = None
    if args.record:
        rp = Path(args.record).expanduser(); rp.parent.mkdir(parents=True, exist_ok=True)
        new = not rp.exists() or rp.stat().st_size == 0
        rec = open(rp, "a", encoding="utf-8")
        if new:
            rec.write("t_wall,t_rel,iface,side,unit,joint,native_deg,logical_deg,yaml_deg\n")
        print(f"[pose] recording to {rp} every {args.interval}s (Ctrl-C to stop)")

    def snapshot() -> None:
        now = time.monotonic() - t0
        wall = time.time()
        for iface in args.ifaces:
            side, unit, _ = UNIT[iface]
            for j in JOINTS:
                v = latest.get((iface, j))
                if v is None or time.monotonic() - v[0] > args.stale:
                    continue
                nat = v[1]; log = nat - offsets.get(iface, {}).get(j, 0.0)
                y = yaml_pose[side].get(j)
                rec.write(f"{wall:.3f},{now:.3f},{iface},{side},{unit},{j},{nat:.3f},{log:.3f},{'' if y is None else f'{y:.2f}'}\n")
        rec.flush()

    def cell(iface: str, j: str) -> tuple[str, str, float | None]:
        v = latest.get((iface, j))
        if v is None or time.monotonic() - v[0] > args.stale:
            return "   -   ", "   -   ", None
        nat = v[1]
        log = nat - offsets.get(iface, {}).get(j, 0.0)
        return f"{nat:7.1f}", f"{log:7.1f}", log

    def render() -> None:
        now = time.monotonic() - t0
        for side in ("left", "right"):
            lead = f"can_lead_{side[0]}"; fol = f"can_fol_{side[0]}"
            if lead not in args.ifaces and fol not in args.ifaces:
                continue
            print(f"== {side}  t={now:6.1f}s   native[deg]: lead / fol      logical[deg]: lead / fol     yaml    Δ(lead-fol)")
            for j in JOINTS:
                ln, ll, lv = cell(lead, j) if lead in args.ifaces else ("   -   ", "   -   ", None)
                fn, fl, fv = cell(fol, j) if fol in args.ifaces else ("   -   ", "   -   ", None)
                y = yaml_pose[side].get(j)
                ys = f"{y:7.2f}" if y is not None else "   -   "
                if lv is not None and fv is not None:
                    d = lv - fv
                    ds = f"{d:+7.1f}" + (" ★" if abs(d) > args.warn else "")
                else:
                    ds = "   -   "
                print(f"   {j:8s} {ln} / {fn}      {ll} / {fl}     {ys}   {ds}")
        print()

    try:
        if args.once:
            time.sleep(1.5)
            render()
        else:
            while not stop.is_set():
                time.sleep(args.interval)
                if rec is not None:
                    snapshot()
                if not args.quiet:
                    render()
    finally:
        stop.set()
        if rec is not None:
            rec.close()
    return 0


if __name__ == "__main__":
    sys.exit(main())
