#!/usr/bin/env python
"""テレオペで「一番長く止めていた姿勢」を開始姿勢にし、そこへ至る経路を立ち上がりの経由地にする。

使い方（Thor・~/projects/openarm-orboh）:
  1. 記録を始める（受信のみ・teleop と並走してよい）:
       python scripts/openarm_pose_monitor.py --record ~/collections/pose-<name>.csv --quiet
  2. 別端末でテレオペ（bash scripts/run_teleop_bimanual.sh）。腕を下ろした構えで数秒止め、
     通したい経路で開始姿勢まで動かし、**開始姿勢で一番長く止める**（目安 10 秒以上）。
  3. 記録を Ctrl-C で止めて、確認だけする:
       python scripts/pose_from_dwell.py ~/collections/pose-<name>.csv
  4. 良ければセットに書き出す（新しいセット名。既存ファイルは上書きしない）:
       python scripts/pose_from_dwell.py ~/collections/pose-<name>.csv --set <name> --write

決め方:
  - 静止 = 両腕の全関節（gripper 除く）の速度が --still-dps 未満。静止が --min-dwell 秒以上続いた区間を拾い、
    一番長い区間の各関節の中央値を開始姿勢にする（両腕が同時に止まっている区間で決める）。
  - 経路 = その直前の静止区間の終わり（無ければ記録の頭）から、開始姿勢の区間の始まりまで。
  - 経由地 = 経路を Ramer-Douglas-Peucker で間引いた点。整列は経由地の間を関節空間で直線補間するだけで
    衝突を見ない（2026-09-23 に直線で机に当たった）。そこで「直線で結んでも、テレオペで実際に通った経路から
    どの関節も --tol-deg 度以内しか外れない」点だけを残す。経路の最初の点（直前の静止姿勢）も経由地に含める。
  - 値は follower の logical（整列が指令する座標と同じ）。gripper は較正ゼロに固定（起動時に非ゼロの把持指令で
    follower gripper を壊した事故による既存の約束）。
"""
from __future__ import annotations

import argparse
import csv
import statistics
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
ARM_JOINTS = [f"joint_{i}" for i in range(1, 8)]


def load(path: Path, unit: str, sides: list[str]) -> tuple[list[float], dict[str, list[list[float]]]]:
    """CSV（縦持ち）を時刻ごとの姿勢列にする。全 side の全関節が揃った時刻だけ残す。"""
    by_t: dict[float, dict[tuple[str, str], float]] = defaultdict(dict)
    for r in csv.DictReader(open(path, encoding="utf-8")):
        if r.get("unit") != unit or r.get("side") not in sides or r.get("joint") not in ARM_JOINTS:
            continue
        try:
            by_t[float(r["t_rel"])][(r["side"], r["joint"])] = float(r["logical_deg"])
        except (TypeError, ValueError):
            continue
    ts: list[float] = []
    q: dict[str, list[list[float]]] = {s: [] for s in sides}
    need = {(s, j) for s in sides for j in ARM_JOINTS}
    for t in sorted(by_t):
        d = by_t[t]
        if not need <= d.keys():
            continue
        ts.append(t)
        for s in sides:
            q[s].append([d[(s, j)] for j in ARM_JOINTS])
    return ts, q


def still_segments(ts, q, sides, still_dps, min_dwell):
    """全 side・全関節の速度が still_dps 未満の連続区間 [i0, i1]（i1 含む）で min_dwell 秒以上のもの。"""
    n = len(ts)
    still = [False] * n
    for i in range(1, n):
        dt = ts[i] - ts[i - 1]
        if dt <= 0:
            continue
        v = max(abs(a - b) / dt for s in sides for a, b in zip(q[s][i], q[s][i - 1]))
        still[i] = v < still_dps
    segs, i = [], 1
    while i < n:
        if still[i]:
            j = i
            while j + 1 < n and still[j + 1]:
                j += 1
            i0 = i - 1  # 速度は i-1→i の区間なので、区間の頭は i-1
            if ts[j] - ts[i0] >= min_dwell:
                segs.append((i0, j))
            i = j + 1
        else:
            i += 1
    return segs


def rdp(points: list[list[float]], tol: float) -> list[int]:
    """関節空間の折れ線を間引く。距離は「直線補間との各関節の差の最大値」（度）。残す点の添字を返す。"""
    if len(points) < 3:
        return list(range(len(points)))
    keep = {0, len(points) - 1}
    stack = [(0, len(points) - 1)]
    while stack:
        a, b = stack.pop()
        pa, pb = points[a], points[b]
        worst, wi = -1.0, -1
        for k in range(a + 1, b):
            u = (k - a) / (b - a)  # 時刻でなく添字で補間（整列の直線補間と同じく単調な 1 本の線とみなす）
            d = max(abs(points[k][m] - (pa[m] + u * (pb[m] - pa[m]))) for m in range(len(pa)))
            if d > worst:
                worst, wi = d, k
        if worst > tol:
            keep.add(wi)
            stack += [(a, wi), (wi, b)]
    return sorted(keep)


def max_dev(points, idx):
    """間引いた線と元の経路の最大ずれ（度）。点ごとに、その点を挟む 2 経由地の直線補間と比べる。"""
    worst = 0.0
    for a, b in zip(idx, idx[1:]):
        for k in range(a + 1, b):
            u = (k - a) / (b - a)
            worst = max(worst, max(abs(points[k][m] - (points[a][m] + u * (points[b][m] - points[a][m])))
                                   for m in range(len(points[a]))))
    return worst


def side_limits(side: str) -> dict[str, tuple[float, float]] | None:
    """lerobot の左右の既定可動域（follower と leader が共有）。lerobot が import できなければ None。"""
    try:
        from lerobot.robots.openarm_follower.config_openarm_follower import (
            LEFT_DEFAULT_JOINTS_LIMITS, RIGHT_DEFAULT_JOINTS_LIMITS)
    except Exception:
        return None
    return LEFT_DEFAULT_JOINTS_LIMITS if side == "left" else RIGHT_DEFAULT_JOINTS_LIMITS


def limit_warnings(side: str, pose: list[float], wps: list[list[float]], path: list[list[float]],
                   margin: float, wrist_max: float) -> list[str]:
    """開始姿勢・経由地が可動域の端から margin 以内、経路中の手首ロール J7 が wrist_max 超を警告する（止めない）。"""
    out: list[str] = []
    lim = side_limits(side)
    if lim is None:
        return ["可動域を読めない（lerobot の venv で実行すると検査できる）"]
    for label, q in [("開始姿勢", pose)] + [(f"経由地 {k}", w) for k, w in enumerate(wps)]:
        for i, v in enumerate(q):
            lo, hi = lim[f"joint_{i + 1}"]
            m = min(v - lo, hi - v)
            if m < margin:
                out.append(f"{label} J{i + 1}={v:.1f} が可動域 [{lo:.0f}, {hi:.0f}] の端まで {m:.1f}°（目安 {margin:.0f}° 以上）")
    j7 = max((abs(p[6]) for p in path), default=0.0)
    if j7 > wrist_max:
        out.append(f"経路中に手首ロール J7 が {j7:.1f}° まで回った（目安 {wrist_max:.0f}° 以下。持ち手をねじると経由地が増え、起動のたびに手首が回る）")
    return out


def fmt_pose(vals: list[float], indent: str = "") -> str:
    return "".join(f"{indent}joint_{i + 1}: {v:.2f}\n" for i, v in enumerate(vals))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", type=Path, help="openarm_pose_monitor.py --record の CSV")
    ap.add_argument("--sides", default="left,right", help="対象の腕（既定 left,right）")
    ap.add_argument("--unit", default="follower", choices=["follower", "leader"])
    ap.add_argument("--still-dps", type=float, default=3.0, help="静止とみなす関節速度の上限 [deg/s]（既定 3）")
    ap.add_argument("--min-dwell", type=float, default=2.0, help="静止区間とみなす最短の長さ [s]（既定 2）")
    ap.add_argument("--tol-deg", type=float, default=5.0, help="経由地を結ぶ直線と実経路の許容ずれ [deg]（既定 5）")
    ap.add_argument("--limit-margin", type=float, default=5.0, help="可動域の端からの余裕の目安 [deg]（下回ると警告・既定 5）")
    ap.add_argument("--wrist-max", type=float, default=60.0, help="経路中の手首ロール J7 の目安 [deg]（超えると警告・既定 60）")
    ap.add_argument("--since", type=float, default=None,
                    help="この t_rel [s] より後だけを見る（同じ記録に起動時の整列が入っているとき、整列中の静止を拾わないため）")
    ap.add_argument("--align-speed", type=float, default=40.0,
                    help="起動時の整列の最大関節速度 [deg/s]（YAML の align.max_speed_dps・既定 40＝公式の 2.2s/約 90° 相当）")
    ap.add_argument("--set", help="書き出し先のセット名（config/pose_sets/<set>/・新しい名前だけ）")
    ap.add_argument("--scene", help="このセットで録るシーン（scenes/<scene>.yaml の name・後から sync でも可）")
    ap.add_argument("--write", action="store_true", help="left/right.yaml を書き出す（無ければ確認表示のみ）")
    args = ap.parse_args()

    sides = [s.strip() for s in args.sides.split(",") if s.strip()]
    ts, q = load(args.csv.expanduser(), args.unit, sides)
    if args.since is not None:
        keep = [i for i, t in enumerate(ts) if t >= args.since]
        ts = [ts[i] for i in keep]
        q = {s: [q[s][i] for i in keep] for s in sides}
    if len(ts) < 10:
        print(f"[dwell] 使える行が少ない（{len(ts)} 時刻）。{args.unit} の {sides} が記録されているか確認する", file=sys.stderr)
        return 1
    segs = still_segments(ts, q, sides, args.still_dps, args.min_dwell)
    if not segs:
        print(f"[dwell] {args.min_dwell}s 以上の静止が無い（--still-dps {args.still_dps}）", file=sys.stderr)
        return 1
    ranked = sorted(segs, key=lambda s: ts[s[1]] - ts[s[0]], reverse=True)
    best = ranked[0]
    print(f"[dwell] 記録 {ts[-1] - ts[0]:.1f}s・静止区間 {len(segs)} 個（長い順）:")
    for s in ranked[:5]:
        mark = "  ← 開始姿勢" if s == best else ""
        print(f"    t={ts[s[0]]:7.1f}〜{ts[s[1]]:7.1f}s  {ts[s[1]] - ts[s[0]]:5.1f}s{mark}")
    if len(ranked) > 1 and (ts[ranked[1][1]] - ts[ranked[1][0]]) > 0.8 * (ts[best[1]] - ts[best[0]]):
        print("[dwell] ⚠ 2 番目の静止区間が 1 番目と近い長さ。意図した姿勢か上の時刻で確かめる")

    # 直前の静止区間のうち、開始姿勢と各関節 --tol-deg 以内のもの（止める直前の微調整）は開始姿勢の一部として飛ばし、
    # それより前の「別の構え」で止めた区間の終わりを経路の頭にする（2026-10-01 実測: 微調整の 3.8s 静止で経路を取りこぼした）
    def seg_pose(seg, side):
        return [statistics.median(c) for c in zip(*q[side][seg[0]:seg[1] + 1])]
    best_pose = {s: seg_pose(best, s) for s in sides}
    def near(seg):
        return all(abs(a - b) <= args.tol_deg for s in sides for a, b in zip(seg_pose(seg, s), best_pose[s]))
    prev = [s for s in segs if s[1] < best[0] and not near(s)]
    p0 = prev[-1][1] if prev else 0
    p1 = best[0]
    print(f"[dwell] 経路 = t={ts[p0]:.1f}〜{ts[p1]:.1f}s（{'直前の静止区間の終わりから' if prev else '記録の頭から'}）")

    out: dict[str, str] = {}
    for s in sides:
        seg = q[s][best[0]:best[1] + 1]
        pose = [statistics.median(c) for c in zip(*seg)]
        spread = max(max(c) - min(c) for c in zip(*seg))
        path = q[s][p0:p1 + 1]
        idx = rdp(path, args.tol_deg) if len(path) >= 2 else []
        # 最後の点（静止に入る瞬間）は開始姿勢そのものとほぼ同じなので経由地から外す
        wps = [path[i] for i in idx[:-1]] if idx else []
        print(f"[dwell] {s}: 開始姿勢（区間内の揺れ最大 {spread:.2f}°）")
        print("        " + " ".join(f"J{i + 1} {v:7.2f}" for i, v in enumerate(pose)))
        print(f"        経由地 {len(wps)} 点（実経路との最大ずれ {max_dev(path, idx) if idx else 0:.1f}° ≤ {args.tol_deg}°）")
        for k, w in enumerate(wps):
            print(f"          {k}: " + " ".join(f"{v:7.2f}" for v in w))
        for wmsg in limit_warnings(s, pose, wps, path, args.limit_margin, args.wrist_max):
            print(f"        ⚠ {wmsg}")
        stamp = datetime.now().strftime("%Y-%m-%d %H:%M")
        body = (f"# OpenArm 開始姿勢 -- side={s}, degrees (logical・{args.unit})。\n"
                f"# {stamp} scripts/pose_from_dwell.py で生成: {args.csv.name} の一番長い静止区間 "
                f"t={ts[best[0]]:.1f}〜{ts[best[1]]:.1f}s（{ts[best[1]] - ts[best[0]]:.1f}s）の中央値。\n"
                f"# waypoints = その直前の経路（t={ts[p0]:.1f}〜{ts[p1]:.1f}s）を RDP で間引いた点（直線と実経路のずれ ≤ {args.tol_deg}°）。\n"
                f"# 整列は現在姿勢から waypoints を順にたどり、最後にこの開始姿勢へ入る（関節空間の直線補間・衝突判定なし）。\n"
                f"# gripper は較正ゼロに固定（起動時の非ゼロ把持指令で follower gripper を壊した事故による約束）。\n")
        # 整列は既定で 1 移動 2.2s 固定なので、経由地が多いと極端に遅い（2026-10-01 実測で 4 腕 約 1 分 45 秒）。
        # 移動量で時間を決めさせる（lerobot fork 46a98170 の align.max_speed_dps）。
        body += f"align:\n  max_speed_dps: {args.align_speed:.1f}\n"
        if wps:
            body += "waypoints:\n" + "".join("  - " + fmt_pose(w, "    ").lstrip() for w in wps)
        body += fmt_pose(pose) + "gripper: 0.0\n"
        out[s] = body

    if not args.write:
        print("[dwell] 確認のみ。書き出すなら --set <新しいセット名> --write")
        return 0
    if not args.set:
        print("[dwell] --write には --set <新しいセット名> が要る", file=sys.stderr)
        return 1
    import os
    d = Path(os.environ.get("PS_ROOT", REPO / "config/pose_sets")) / args.set
    if d.exists():
        print(f"[dwell] 既にある: {d}（セットは作り直さない。別名で撮る）", file=sys.stderr)
        return 1
    d.mkdir(parents=True)
    for s in sides:
        (d / f"{s}.yaml").write_text(out[s], encoding="utf-8")
        print(f"[dwell] 書き出し: {d / f'{s}.yaml'}")
    # 下書き（draft）のセットとして登録する。左右を個別に決めた姿勢なので鏡像検算は外す。
    cmd = [sys.executable, str(REPO / "scripts/pose_set.py"), "new", args.set]
    if len(sides) == 2:
        cmd += ["--mirror", "false"]
    if args.scene:
        cmd += ["--scene", args.scene]
    import subprocess
    rc = subprocess.run(cmd).returncode
    if rc == 0:
        print(f"[dwell] 使うなら: python3 scripts/pose_set.py current {args.set}")
    return rc


if __name__ == "__main__":
    sys.exit(main())
