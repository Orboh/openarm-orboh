#!/usr/bin/env python3
"""
rollout_timing_stats.py — 試走の traj.csv から「止まり方」「つなぎ目の飛び」「グリッパの往復」を測る（2026-10-07）

推論の方式（直列・非同期・RTC）を比べるときの物差し。eval_openarm.py の traj.csv だけを読む（numpy 不要）。
Thor の ~/rollouts/<run>/ でも、Vault の rollouts/<試験>/<run>/（traj.csv を写したもの）でも動く。

使い方:
    python3 scripts/g4/rollout_timing_stats.py <run_dir> [<run_dir> ...]
    python3 scripts/g4/rollout_timing_stats.py --by-index <run_dir>     # チャンクの位置ごとの 1 step 変化も出す
    python3 scripts/g4/rollout_timing_stats.py --trace <run_dir>        # 1 秒ごとの実際の動きと指令との差

出す値:
  - 指令の間隔（行の t_sec の差）: 100ms を超えた回数と最大＝腕が止まった回数と長さ
  - 境目の指令の飛び（腕の関節の最大）: 直列は chunk_i == 0 の行、新しいスケジューラは switched == 1 の行
  - 実関節の 2 階差分（腕の関節の最大）: 実際のがくっとした動きの大きさ
  - グリッパの指令の総変化（度/秒）と、1 step で 20 度を超える飛びの回数＝計画の乗り換えによる往復
  - 指令と現在角の差: 腕が指令に付いてきているか（dry では腕が動かないので大きくなる）
  - underrun（新しいスケジューラのみ）: 指令が尽きて最後の指令を保った周期
"""

import argparse
import collections
import csv
import glob
import os
import statistics as st


def q(v, p):
    v = sorted(v)
    return v[int(p * (len(v) - 1))] if v else float("nan")


def load(run_dir):
    fs = glob.glob(os.path.join(run_dir, "**", "traj.csv"), recursive=True)
    if not fs:
        return None
    return list(csv.DictReader(open(fs[0])))


def summarize(run_dir, by_index=False):
    r = load(run_dir)
    name = os.path.basename(os.path.normpath(run_dir))
    if not r or len(r) < 3:
        print(f"{name}: traj.csv が無いか、行が少ない")
        return
    t = [float(x["t_sec"]) for x in r]
    gaps = [(t[i] - t[i - 1]) * 1000 for i in range(1, len(t))]
    ck = [k for k in r[0] if k.startswith("cmd.") and "gripper" not in k]
    pk = [k.replace("cmd.", "present.") for k in ck]
    sched = "switched" in r[0]
    bnd = [i for i in range(1, len(r)) if (r[i]["switched"] == "1" if sched else int(r[i]["chunk_i"]) == 0)]
    jump = [max(abs(float(r[i][k]) - float(r[i - 1][k])) for k in ck) for i in bnd]
    pres = [[float(x[k]) for k in pk] for x in r]
    acc = [max(abs(pres[i + 1][j] - 2 * pres[i][j] + pres[i - 1][j]) for j in range(len(pk))) for i in range(1, len(pres) - 1)]
    dev = [max(abs(float(x[c]) - float(x[p])) for c, p in zip(ck, pk)) for x in r]
    dur = t[-1] - t[0]
    und = sum(int(x["underrun"]) for x in r) if sched else 0
    print(f"== {name}（{'新しいスケジューラ' if sched else '直列'}・{len(r)} 行・{dur:.1f} 秒）")
    print(f"  指令の間隔: 中央値 {st.median(gaps):.0f}ms・100ms 超 {sum(g > 100 for g in gaps)} 回・最大 {max(gaps):.0f}ms・underrun {und}")
    if jump:
        print(f"  境目の指令の飛び（腕・{len(bnd)} 回）: 中央値 {st.median(jump):.2f}・p99 {q(jump, .99):.2f}・最大 {max(jump):.2f} 度")
    print(f"  実関節の 2 階差分: 中央値 {st.median(acc):.2f}・p99 {q(acc, .99):.2f}・最大 {max(acc):.2f} 度")
    print(f"  指令と現在角の差: 中央値 {st.median(dev):.1f}・p90 {q(dev, .9):.1f}・最大 {max(dev):.1f} 度")
    for g in ("left_gripper.pos", "right_gripper.pos"):
        if "cmd." + g not in r[0]:
            continue
        c = [float(x["cmd." + g]) for x in r]
        tv = sum(abs(c[i] - c[i - 1]) for i in range(1, len(c)))
        big = sum(1 for i in range(1, len(c)) if abs(c[i] - c[i - 1]) > 20)
        print(f"  {g}: 指令の総変化 {tv / dur:.1f} 度/秒・1 step で 20 度超 {big} 回")
    if by_index:
        by = collections.defaultdict(list)
        for i in range(1, len(r)):
            key = "switch" if (sched and r[i]["switched"] == "1") else f"idx{int(r[i]['chunk_i']):02d}"
            by[key].append(max(abs(float(r[i][k]) - float(r[i - 1][k])) for k in ck))
        print("  腕の指令の 1 step 変化（チャンクの位置別・中央値/最大・5 行以上の位置のみ）:")
        for k in sorted(by):
            if len(by[k]) >= 5:
                print(f"    {k}: n={len(by[k])} {st.median(by[k]):.2f}/{max(by[k]):.2f}")


def trace(run_dir):
    r = load(run_dir)
    pk = [k for k in r[0] if k.startswith("present.") and "gripper" not in k]
    ck = [k.replace("present.", "cmd.") for k in pk]
    prev = None
    for i in range(0, len(r), 30):
        x = r[i]
        p = [float(x[k]) for k in pk]
        mv = max(abs(a - b) for a, b in zip(p, prev)) if prev else 0.0
        dv = max(abs(float(x[c]) - float(x[k])) for c, k in zip(ck, pk))
        print(f"t={float(x['t_sec']):6.1f} step={x['step']:>5} 1 秒の実際の動き {mv:5.1f} 指令との差 {dv:5.1f}")
        prev = p


if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="+")
    ap.add_argument("--by-index", action="store_true")
    ap.add_argument("--trace", action="store_true")
    a = ap.parse_args()
    for d in a.runs:
        if a.trace:
            print(f"== {os.path.basename(os.path.normpath(d))}")
            trace(d)
        else:
            summarize(d, a.by_index)
