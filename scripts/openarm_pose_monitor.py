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


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("ifaces", nargs="*", default=list(UNIT), help="socketcan IF（既定: 4 バス）")
    ap.add_argument("--interval", type=float, default=1.0, help="表示間隔 [s]")
    ap.add_argument("--once", action="store_true", help="1.5 秒集めて 1 回出して終了")
    ap.add_argument("--warn", type=float, default=3.0, help="|Δ| がこの角度 [deg] を超えた行に ★")
    ap.add_argument("--stale", type=float, default=1.0, help="この秒数より古い値は '-' 表示")
    args = ap.parse_args()

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
                render()
    finally:
        stop.set()
    return 0


if __name__ == "__main__":
    sys.exit(main())
