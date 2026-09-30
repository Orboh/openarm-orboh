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

ライブ表示中は Enter で目印（MARK）を打てる: その時刻の行が [MARK n] として表示され、--record 中なら CSV にも
  iface=MARK の行が入り、--report の末尾に「目印 n の直前の姿勢」が出る。手先を目で見て「いま曲がった」と
  思った瞬間に押す。

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
    """--record した CSV から、直近の「整列（位置制御）→ loop 開始」の遷移を side ごとに要約する。

    各バスの受信を 1 s 以上の空白で区切り（整列中は自ユニットだけ polling される・接続待ちは無受信）、
    4 バスがすべて受信中になる最後の区間の先頭を loop 開始とする。各ユニットの整列終了値は
    loop 開始より前の最後の区間の末尾の値。loop 開始 +0.1 / +1 / +3 / +8 s の値を並べ、
    YAML からのずれ・leader≠follower・loop 後の動き（崩れ）に注記を付ける。
    """
    import csv
    from collections import defaultdict

    rows = list(csv.DictReader(open(path, encoding="utf-8")))
    if not rows:
        print("empty"); return 1
    by: dict = defaultdict(list)  # (iface, joint) -> [(t, logical, yaml)]
    for r in rows:
        try:
            by[(r["iface"], r["joint"])].append((float(r["t_rel"]), float(r["logical_deg"]),
                                                float(r["yaml_deg"]) if r.get("yaml_deg") else None))
        except (KeyError, ValueError):
            continue
    buses = [i for i in UNIT if (i, "joint_1") in by]
    if not buses:
        print("no data"); return 1

    def segments(iface: str) -> list[tuple[float, float]]:
        ts = sorted(t for (t, _, _) in by[(iface, "joint_1")])
        segs: list[tuple[float, float]] = []; s0 = ts[0]; prev = ts[0]
        for t in ts[1:]:
            if t - prev > 1.0:
                segs.append((s0, prev)); s0 = t
            prev = t
        segs.append((s0, prev))
        return segs

    segs = {i: segments(i) for i in buses}
    for i in buses:
        print(f"{i:10s} 受信区間: {[(round(a, 1), round(b, 1)) for a, b in segs[i][-4:]]}")
    t_loop = max(segs[i][-1][0] for i in buses)
    print(f"loop 開始（全バス受信）≈ {t_loop:.1f} s")

    def at(iface: str, j: str, t: float, w: float = 0.3) -> float | None:
        vals = [l for (tt, l, _) in by.get((iface, j), []) if t - w <= tt <= t + w]
        return sum(vals) / len(vals) if vals else None

    def last_before(iface: str, j: str, t: float) -> float | None:
        vals = [l for (tt, l, _) in by.get((iface, j), []) if tt < t]
        return vals[-1] if vals else None

    def align_end(iface: str) -> float | None:
        prior = [s for s in segs.get(iface, []) if s[0] < t_loop - 0.5]
        return prior[-1][1] if prior else None

    def f(v: float | None) -> str:
        return "   -  " if v is None else f"{v:6.1f}"

    for side in ("left", "right"):
        lead = f"can_lead_{side[0]}"; fol = f"can_fol_{side[0]}"
        if lead not in buses and fol not in buses:
            continue
        ae_l = align_end(lead) if lead in buses else None
        ae_f = align_end(fol) if fol in buses else None
        yaml = {j: next((y for (_, _, y) in by.get((lead if lead in buses else fol, j), []) if y is not None), None) for j in JOINTS}
        print(f"== {side}  leader 整列終了≈{f(ae_l)} s  follower 整列終了≈{f(ae_f)} s  loop={t_loop:.1f} s   （logical [deg]）")
        print("   joint     yaml | lead@align fol@align     Δ | lead+0.1 fol+0.1 | lead+1  fol+1  | lead+3  fol+3  | lead+8  fol+8  | 判定")
        for j in JOINTS:
            la = last_before(lead, j, ae_l + 0.01) if ae_l is not None else None
            fa = last_before(fol, j, ae_f + 0.01) if ae_f is not None else None
            d = (la - fa) if (la is not None and fa is not None) else None
            l1, f1 = at(lead, j, t_loop + 0.1), at(fol, j, t_loop + 0.1)
            l2, f2 = at(lead, j, t_loop + 1.0), at(fol, j, t_loop + 1.0)
            l3, f3 = at(lead, j, t_loop + 3.0), at(fol, j, t_loop + 3.0)
            l8, f8 = at(lead, j, t_loop + 8.0), at(fol, j, t_loop + 8.0)
            y = yaml.get(j)
            notes = []
            if y is not None and la is not None and abs(la - y) > warn: notes.append(f"leader 整列が YAML から {la - y:+.1f}")
            if y is not None and fa is not None and abs(fa - y) > warn: notes.append(f"follower 整列が YAML から {fa - y:+.1f}")
            if d is not None and abs(d) > warn: notes.append("整列時 lead≠fol ★")
            if l1 is not None and l3 is not None and abs(l3 - l1) > 5: notes.append(f"loop 後 leader {l3 - l1:+.1f}（崩れ）")
            print(f"   {j:8s} {f(y)} | {f(la):>10s} {f(fa):>9s} {f(d):>5s} | {f(l1)} {f(f1)} | {f(l2)} {f(f2)} | {f(l3)} {f(f3)} | {f(l8)} {f(f8)} | {'; '.join(notes)}")
        print()
    marks = sorted((float(r["t_rel"]), r.get("joint", "")) for r in rows if r.get("iface") == "MARK")
    for tm, name in marks:
        print(f"== 目印 {name} t={tm:.1f}s（loop 開始から {tm - t_loop:+.1f}s）の直前 0.3 s の logical [deg]")
        for side in ("left", "right"):
            lead = f"can_lead_{side[0]}"; fol = f"can_fol_{side[0]}"
            if lead not in buses and fol not in buses:
                continue
            row = "  ".join(f"{j[-1] if j != 'gripper' else 'g'}:{f(at(lead, j, tm - 0.15, 0.15)).strip()}/{f(at(fol, j, tm - 0.15, 0.15)).strip()}" for j in JOINTS)
            print(f"   {side:5s} lead/fol  {row}")
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
    # 比較対象の開始姿勢は開始姿勢セット（env POSE_SET・未指定なら config/pose_sets/CURRENT）
    ps_root = REPO / "config/pose_sets"
    ps_name = os.environ.get("POSE_SET") or (ps_root / "CURRENT").read_text().strip()
    yaml_pose = {"left": load_yaml_pose(ps_root / ps_name / "left.yaml"),
                 "right": load_yaml_pose(ps_root / ps_name / "right.yaml")}
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
    marks: list[tuple[float, int]] = []

    def _stdin_marks() -> None:
        n = 0
        try:
            for _ in sys.stdin:
                n += 1
                tm = time.monotonic() - t0
                marks.append((tm, n))
                print(f"[MARK {n}] t={tm:.1f}s", flush=True)
                if rec is not None:
                    rec.write(f"{time.time():.3f},{tm:.3f},MARK,,,mark_{n},,,\n"); rec.flush()
        except Exception:
            pass

    if not args.once and sys.stdin and sys.stdin.isatty():
        threading.Thread(target=_stdin_marks, daemon=True).start()
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
