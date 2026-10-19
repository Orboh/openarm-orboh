#!/usr/bin/env python3
"""openarm_pose_monitor.py --record の CSV から、follower 左右の J1/J4/J5/J6/J7 の時系列を 0.5° 以上動いた行だけ出す。

起動時の整列の終わり（目標との差）・ループ開始の飛び・整列中の往復振動を目で読むための補助（2026-10-01 板橋）。
動きから整列の境界を自動判定する試み（loop-start jump 検出）は誤検出したので持たない。
  python3 scripts/pose_monitor_traj.py ~/collections/pose-align-sag-before-20261001.csv [t_rel の上限]
"""
import csv,collections,sys
F=sys.argv[1]; tmax=float(sys.argv[2]) if len(sys.argv)>2 else 1e9
by=collections.defaultdict(lambda: collections.defaultdict(dict))
for r in csv.DictReader(open(F)):
    if r["iface"]=="MARK" or not r["logical_deg"]: continue
    t=float(r["t_rel"])
    if t<=tmax: by[(r["unit"],r["side"])][t][r["joint"]]=float(r["logical_deg"])
for u in [("follower","left"),("follower","right")]:
    d=by[u]; ts=sorted(t for t in d if len(d[t])>=7)
    print("==",u,"n",len(ts), "t %.1f..%.1f"%(ts[0],ts[-1]) if ts else "")
    prev=None
    for t in ts:
        v=[d[t][f"joint_{i}"] for i in (1,4,5,6,7)]
        if prev is None or max(abs(a-b) for a,b in zip(v,prev))>0.5:
            print(f"   {t:6.1f} J1 {v[0]:6.1f} J4 {v[1]:6.1f} J5 {v[2]:5.1f} J6 {v[3]:5.1f} J7 {v[4]:6.1f}")
        prev=v
