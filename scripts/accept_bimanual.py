#!/usr/bin/env python3
"""両腕 OpenArm 収集（LeRobot v3.0 データセット）の受入検査。測定値と pass/flag/fail の**案**を出す。

設計正本: orboh-company-vault
  07Member's-free-space/Kento/02-Tech-Plan/02VLA/itabashi-bimanual/itabashi-bimanual-acceptance-check.md

何を見るか（観点 A〜F。閾値は「案」で、最終判断は責任者が持つ）:
  A 完全性   フレーム数（data / meta / 動画区間）の一致、frame_index の連番、timestamp = frame_index/fps、NaN/inf
  B カメラ   console.log の stale frame・record loop の fps 超過（over budget）・CAN drops を録画区間だけに帰属
             --with-video で mp4 をデコードし、左腕が動いている区間だけの重複フレーム率と最長連続重複を出す
  C 開始条件 start-pose-*.jsonl の → 時点のずれ（worst joint）と、data 先頭フレームの state を参照姿勢と比べたずれ
  D 右腕固定 右 7 関節＋gripper の action が完全一定か、state の定常区間のばらつき、先頭の過渡（settle）フレーム数、
             エピソード間の定常姿勢のずれ（カメラ役なので動いていたら問題）
  E 左腕実演 leader(action) と follower(state) の追従誤差、gripper 開閉回数（3 個積むなら close >= 3）、速度の外れ値、
             idle（静止）の長さ
  F 分布     エピソード長・左腕の可動範囲を baseline（既定 = 最初の run のエピソード）と比べる

使い方（Vector に写したデータでも、Thor の HF キャッシュ直でも同じ。Thor で回すときは s* の録画中を避ける）:
  python scripts/accept_bimanual.py \
      --root ~/.cache/huggingface/lerobot/Orboh/<dataset> \
      --run ~/collections/<run-s1> --run ~/collections/<run-s2> --run ~/collections/<run-s3> \
      --episodes 0-29 [--baseline 0-9] [--with-video] [--json out.json] [--verbose]

  --episodes  対象の episode_index（"0-29" や "0-9,20-29"）。省略時は meta にある全エピソード。
              **録画中のデータセットを読むときは必ず指定する**（30 本目以降の行が混ざるので）。
  --baseline  観点 F の比較基準。省略時は最初の --run に属するエピソード（gate）。
  --with-video mp4 を全デコードする（pyav が要る。無ければ観点 B の動画項目を skip する）。既定 off。
  --json      全測定値を JSON にも書く。

依存: pandas / pyarrow / numpy（Thor の lerobot venv にある）。--with-video のときだけ av（pyav）。

閾値の既定値（すべて案。引数で上書きできる）:
  --tol-start-pass 3.0    --tol-start-flag 15.0    [deg]  → 時点・先頭フレームの左腕ずれ（worst joint）。3 = record の START_POSE_TOL_DEG
  --tol-right-range 0.5 / 2.0          [deg]  右腕 state の定常レンジ（エピソード後半 50%。pass / flag。超えたら fail）。1° ≒ 手先 0.5 m で 9 mm の画ずれ
  --tol-right-settle 0 / 30            [frames] 先頭で右腕が定常から外れているフレーム数（pass / flag）。30 = 1 秒
  --tol-right-mean 0.5 / 2.0           [deg]  エピソード間の右腕定常姿勢のずれ（pass / flag）
  --tol-track-p95 5.0 / 10.0           [deg]  左腕 |action - state| の p95（関節ごとの最大）
  --tol-track-max 15.0 / 30.0          [deg]  同 max。CAN 落ちで follower が止まると leader だけ進んで大きく開く
  --min-grip-holds 2                          物を保持した区間（state が -55..-8 に 0.3 s 以上）の数。2 個運ぶ = 2 回以上
  --grip-hold-range -55 -8 / --grip-hold-min-s 0.3   保持区間の定義（-65 全開・0 空で全閉・積み木幅 ≒ -30）
  --grip-threshold -40.0               [deg]  gripper の開/閉イベントの境（観測ヒストグラムの谷）
  --tol-speed 200.0 / 450.0            [deg/s] 左腕関節速度の max（pass / flag。450 = 1 フレームで 15° 跳ぶ = 計測グリッチ相当で fail）
  --tol-idle 10.0                      [s]    最長の静止区間（flag）
  --tol-overbudget 0.02 / 0.05                fps 超過サイクル数 / フレーム数（pass / flag）。1 秒に 1 サイクル遅れ = 3.3% が警告が毎秒出る水準
  --tol-minhz 20.0                     [Hz]   最遅サイクルの Hz（未満で flag）
  --tol-len-ratio 1.5                         エピソード長 / baseline 中央値（超えたら flag）
  --tol-range-exceed 5.0 / 15.0        [deg]  左腕可動範囲が baseline の包絡を超えた量（pass / flag）
  --tol-dup 0.05 / 0.10                       動画: 左腕が動いている区間の重複フレーム率（wrist_left 基準。pass / flag）
  --tol-dup-run 3 / 15                 [frames] 動画: 最長連続重複（flag / fail）。15 = 0.5 秒の静止画
  --settle-tol 0.5 [deg] / --settle-grip-tol 5.0 [deg] 右腕の過渡判定に使う定常からの許容（跳ねの尾 ±0.5〜1° も過渡に含める）
  --systematic-frac 0.5                      この割合以上のエピソードに共通する理由は「系統的所見」に分け、grade(系統除く) から外す
  --moving-speed 5.0 [deg/s]           「左腕が動いている」の境

出力: 標準出力に pipe テーブル（run の整合、エピソード表、セッション集計、閾値一覧）。
git add / commit はしない（レビューして呼び出し元が commit する）。
"""

from __future__ import annotations

import argparse
import glob
import json
import math
import os
import re
import sys
from collections import OrderedDict
from pathlib import Path

import numpy as np
import pandas as pd
import pyarrow.parquet as pq

# ---------------------------------------------------------------------------
# 引数
# ---------------------------------------------------------------------------


def parse_ranges(spec: str | None) -> list[int] | None:
    if not spec:
        return None
    out: list[int] = []
    for part in spec.split(","):
        part = part.strip()
        if not part:
            continue
        if "-" in part:
            a, b = part.split("-", 1)
            out.extend(range(int(a), int(b) + 1))
        else:
            out.append(int(part))
    return sorted(set(out))


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--root", required=True, help="LeRobot v3.0 データセットの root（meta/ data/ videos/ を持つ）")
    p.add_argument("--run", action="append", default=[], help="収集 run ディレクトリ（console.log / start-pose-*.jsonl / command.sh / manifest.json）。複数可")
    p.add_argument("--episodes", default=None, help='対象 episode_index。例 "0-29" / "0-9,20-29"。省略時は meta の全件')
    p.add_argument("--baseline", default=None, help="観点 F の基準にする episode_index（同じ書式）。省略時は最初の --run のエピソード")
    p.add_argument("--with-video", action="store_true", help="mp4 を全デコードして重複フレーム率を出す（pyav 必須）")
    p.add_argument("--json", default=None, help="全測定値を JSON で書き出すパス")
    p.add_argument("--verbose", action="store_true", help="関節別などの詳細表も出す")
    # 閾値（案）
    p.add_argument("--tol-start-pass", type=float, default=3.0)
    p.add_argument("--tol-start-flag", type=float, default=15.0)
    p.add_argument("--tol-right-range", type=float, nargs=2, default=[0.5, 2.0])
    p.add_argument("--tol-right-settle", type=int, nargs=2, default=[0, 30])
    p.add_argument("--tol-right-mean", type=float, nargs=2, default=[0.5, 2.0])
    p.add_argument("--tol-track-p95", type=float, nargs=2, default=[5.0, 10.0])
    p.add_argument("--tol-track-max", type=float, nargs=2, default=[15.0, 30.0])
    p.add_argument("--min-grip-holds", type=int, default=2)
    p.add_argument("--grip-hold-range", type=float, nargs=2, default=[-55.0, -8.0], help="「物を保持している」とみなす gripper state の範囲（-65 全開・0 空で全閉。積み木幅 ≒ -30）")
    p.add_argument("--grip-hold-min-s", type=float, default=0.3)
    p.add_argument("--grip-threshold", type=float, default=-40.0)
    p.add_argument("--tol-speed", type=float, nargs=2, default=[200.0, 450.0])
    p.add_argument("--tol-idle", type=float, default=10.0)
    p.add_argument("--tol-overbudget", type=float, nargs=2, default=[0.02, 0.05])
    p.add_argument("--tol-minhz", type=float, default=20.0)
    p.add_argument("--tol-len-ratio", type=float, default=1.5)
    p.add_argument("--tol-range-exceed", type=float, nargs=2, default=[5.0, 15.0])
    p.add_argument("--tol-dup", type=float, nargs=2, default=[0.05, 0.10])
    p.add_argument("--tol-dup-run", type=int, nargs=2, default=[3, 15])
    p.add_argument("--settle-tol", type=float, default=0.5)
    p.add_argument("--settle-grip-tol", type=float, default=5.0)
    p.add_argument("--moving-speed", type=float, default=5.0)
    p.add_argument("--dup-diff", type=float, default=0.3, help="重複フレームとみなす前フレームとの平均絶対差（8bit gray・1/4 縮小）")
    p.add_argument("--systematic-frac", type=float, default=0.5, help="この割合以上のエピソードに出る flag/fail 理由は「系統的所見」として分け、grade(系統除く) から外す")
    return p


# ---------------------------------------------------------------------------
# 読み込み
# ---------------------------------------------------------------------------


def load_dataset(root: Path, episodes: list[int] | None):
    info = json.loads((root / "meta" / "info.json").read_text())
    fps = int(info["fps"])
    names = info["features"]["action"]["names"]
    cams = [k.replace("observation.images.", "") for k in info["features"] if k.startswith("observation.images.")]
    ep_files = sorted(glob.glob(str(root / "meta" / "episodes" / "**" / "*.parquet"), recursive=True))
    epm = pd.concat([pq.read_table(f).to_pandas() for f in ep_files], ignore_index=True)
    epm = epm.sort_values("episode_index").reset_index(drop=True)
    if episodes is not None:
        epm = epm[epm["episode_index"].isin(episodes)].reset_index(drop=True)
    want = set(int(e) for e in epm["episode_index"])
    # data: 対象エピソードを含むファイルだけ読む（録画中の file-N は episode_index で切る）
    need_files = set()
    for _, r in epm.iterrows():
        need_files.add(str(root / info["data_path"].format(chunk_index=int(r["data/chunk_index"]), file_index=int(r["data/file_index"]))))
    frames = []
    for f in sorted(need_files):
        if not os.path.exists(f):
            print(f"[accept] WARNING data file missing: {f}", file=sys.stderr)
            continue
        t = pq.read_table(f, columns=["action", "observation.state", "timestamp", "frame_index", "episode_index", "index"]).to_pandas()
        t = t[t["episode_index"].isin(want)]
        frames.append(t)
    data = pd.concat(frames, ignore_index=True).sort_values(["episode_index", "frame_index"]).reset_index(drop=True)
    return info, fps, names, cams, epm, data


def load_run(run_dir: Path) -> dict:
    run: dict = {"dir": str(run_dir), "name": run_dir.name}
    m = re.search(r"-(s\d+|dry|gate)$", run_dir.name)
    run["session"] = m.group(1) if m else run_dir.name
    # command.sh の解決値
    cmd = (run_dir / "command.sh").read_text(errors="replace") if (run_dir / "command.sh").exists() else ""
    def _env(key: str) -> str | None:
        mm = re.search(rf"(?:^|\s){key}=((?:\\.|[^\s\\])*)", cmd)
        return mm.group(1).replace("\\", "") if mm else None
    run["episode_time_s"] = float(_env("EPISODE_TIME_S")) if _env("EPISODE_TIME_S") else None
    run["freeze_side"] = _env("FREEZE_SIDE") or ""
    run["frozen_gripper_deg"] = _env("FROZEN_GRIPPER_DEG")
    run["arm_kp"] = _env("ARM_KP")
    run["gripper_kp"] = _env("GRIPPER_KP")
    run["start_pose_tol_deg"] = _env("START_POSE_TOL_DEG")
    # manifest
    man_p = run_dir / "manifest.json"
    if man_p.exists():
        man = json.loads(man_p.read_text())
        run["scene_sha"] = (man.get("scene") or {}).get("sha256", "")[:12]
        run["scene_name"] = (man.get("scene") or {}).get("name", "")
        run["calib_sha"] = {c["id"]: c["sha256"][:12] for c in man.get("calibration", [])}
        cv = man.get("control_variables") or {}
        run["cameras"] = cv.get("cameras", {})
        run["fps"] = cv.get("fps")
        run["exit_code"] = (man.get("artifacts") or {}).get("exit_code")
        run["operator"] = (man.get("people") or {}).get("operator")
        run["started_local"] = (man.get("identity") or {}).get("started_local")
    # start-pose jsonl
    run["reference"] = None
    run["checked_joints"] = []
    run["start_entries"] = []
    for jp in sorted(run_dir.glob("start-pose-*.jsonl")):
        for line in jp.read_text().splitlines():
            line = line.strip()
            if not line:
                continue
            try:
                e = json.loads(line)
            except json.JSONDecodeError:
                continue
            if e.get("type") == "start_pose_reference":
                run["reference"] = e.get("target_pos", {})
                run["checked_joints"] = e.get("checked_joints", [])
                run["jsonl_tolerance_deg"] = e.get("tolerance_deg")
            elif e.get("type") == "episode":
                run["start_entries"].append(e)
    # console.log: 録画区間に帰属させる
    run["console"] = parse_console(run_dir / "console.log")
    return run


_RE_TIME = re.compile(r"^(?:INFO|WARNING|ERROR) (\d{4}-\d\d-\d\d \d\d:\d\d:\d\d)")
_RE_REC = re.compile(r"Recording episode (\d+)")
_RE_END = re.compile(r"(Right arrow key pressed\. Exiting loop|Left arrow key pressed)")
_RE_OB = re.compile(r"over budget in (\d+)/(\d+) cycles.*?slowest ([\d.]+) Hz")
_RE_DROPS = re.compile(r"drops \[(.*?)\]")


def parse_console(path: Path) -> dict[int, dict]:
    """`Recording episode N` から右/左矢印までを録画区間として、警告を N に帰属させる。
    同じ N が複数回出たら（左矢印で破棄→録り直し）takes を増やし、統計は最後の take のもの。"""
    stats: dict[int, dict] = {}
    if not path.exists():
        return stats
    cur: int | None = None
    last_ts = None
    for line in path.open(errors="replace"):
        mt = _RE_TIME.match(line)
        if mt:
            last_ts = mt.group(1)
        m = _RE_REC.search(line)
        if m:
            cur = int(m.group(1))
            st = stats.setdefault(cur, {"takes": 0})
            st["takes"] += 1
            st.update(ob_lines=0, slow_cycles=0, window_cycles=0, min_hz=float("inf"), stale=0, drops=0, start_ts=last_ts, end_ts=None, end_kind=None)
            continue
        if cur is not None and _RE_END.search(line):
            stats[cur]["end_ts"] = last_ts
            stats[cur]["end_kind"] = "discard" if "Left" in line else "keep"
            cur = None
            continue
        if cur is None:
            continue
        st = stats[cur]
        m = _RE_OB.search(line)
        if m:
            st["ob_lines"] += 1
            st["slow_cycles"] += int(m.group(1))
            st["window_cycles"] += int(m.group(2))
            st["min_hz"] = min(st["min_hz"], float(m.group(3)))
        if "stale frame" in line:
            st["stale"] += 1
        md = _RE_DROPS.search(line)
        if md and md.group(1).strip() != "none":
            st["drops"] += 1
    return stats


# ---------------------------------------------------------------------------
# 判定ヘルパ
# ---------------------------------------------------------------------------

ORDER = {"pass": 0, "flag": 1, "fail": 2, "skip": -1, "n/a": -1}


def grade(value: float | None, pass_max: float, flag_max: float | None = None) -> str:
    if value is None or (isinstance(value, float) and math.isnan(value)):
        return "n/a"
    if value <= pass_max:
        return "pass"
    if flag_max is None or value <= flag_max:
        return "flag"
    return "fail"


def worst(*grades: str) -> str:
    g = [x for x in grades if x in ("pass", "flag", "fail")]
    return max(g, key=lambda x: ORDER[x]) if g else "n/a"


def longest_run(mask: np.ndarray) -> int:
    best = c = 0
    for x in mask:
        c = c + 1 if x else 0
        best = max(best, c)
    return best


def fmt(v, nd=1):
    if v is None:
        return "-"
    if isinstance(v, float):
        if math.isnan(v):
            return "-"
        return f"{v:.{nd}f}"
    return str(v)


def table(headers: list[str], rows: list[list], out=sys.stdout):
    print("| " + " | ".join(headers) + " |", file=out)
    print("|" + "|".join(["---"] * len(headers)) + "|", file=out)
    for r in rows:
        print("| " + " | ".join(str(x) for x in r) + " |", file=out)
    print(file=out)


# ---------------------------------------------------------------------------
# 動画
# ---------------------------------------------------------------------------


def video_framediffs(root: Path, info: dict, epm: pd.DataFrame, cams: list[str]) -> dict[str, np.ndarray] | None:
    try:
        import av  # noqa: F401
    except ImportError:
        print("[accept] --with-video: pyav (av) が import できないので動画項目は skip", file=sys.stderr)
        return None
    import av

    out: dict[str, np.ndarray] = {}
    files = set()
    for _, r in epm.iterrows():
        for cam in cams:
            key = f"observation.images.{cam}"
            files.add((cam, int(r[f"videos/{key}/chunk_index"]), int(r[f"videos/{key}/file_index"])))
    for cam, ci, fi in sorted(files):
        path = root / info["video_path"].format(video_key=f"observation.images.{cam}", chunk_index=ci, file_index=fi)
        if not path.exists():
            print(f"[accept] WARNING video missing: {path}", file=sys.stderr)
            continue
        c = av.open(str(path))
        s = c.streams.video[0]
        s.thread_type = "AUTO"
        prev = None
        diffs: list[float] = []
        for fr in c.decode(s):
            g = fr.to_ndarray(format="gray")[::4, ::4].astype(np.int16)
            diffs.append(float(np.abs(g - prev).mean()) if prev is not None else float("nan"))
            prev = g
        c.close()
        out[f"{cam}/{ci}/{fi}"] = np.asarray(diffs, dtype=np.float32)
        print(f"[accept] decoded {cam} chunk-{ci:03d}/file-{fi:03d}: {len(diffs)} frames", file=sys.stderr)
    return out


# ---------------------------------------------------------------------------
# 本体
# ---------------------------------------------------------------------------


def main(argv=None) -> int:
    args = build_parser().parse_args(argv)
    root = Path(args.root).expanduser()
    episodes = parse_ranges(args.episodes)
    info, fps, names, cams, epm, data = load_dataset(root, episodes)
    ep_list = [int(e) for e in epm["episode_index"]]
    runs = [load_run(Path(r).expanduser()) for r in args.run]

    # episode -> run（jsonl の started* エントリで結ぶ。無ければ console の Recording episode で結ぶ）
    ep_run: dict[int, dict] = {}
    for run in runs:
        for e in run["start_entries"]:
            if str(e.get("outcome", "")).startswith("started"):
                ep_run[int(e["episode_index"])] = run
        for e in run["console"]:
            if run["console"][e].get("end_kind") == "keep":
                ep_run.setdefault(int(e), run)

    baseline = parse_ranges(args.baseline)
    if baseline is None and runs:
        baseline = [e for e in ep_list if ep_run.get(e) is runs[0]]
    if not baseline:
        baseline = ep_list
    baseline = [e for e in baseline if e in ep_list]

    # 右腕/左腕の列
    freeze = ""
    for run in runs:
        if run.get("freeze_side"):
            freeze = run["freeze_side"]
    frozen_side = freeze or "right"
    live_side = "left" if frozen_side == "right" else "right"
    idx_frozen = [i for i, n in enumerate(names) if n.startswith(frozen_side + "_")]
    idx_live_j = [i for i, n in enumerate(names) if n.startswith(live_side + "_joint")]
    idx_live_g = [i for i, n in enumerate(names) if n.startswith(live_side + "_gripper")]
    idx_live_g = idx_live_g[0] if idx_live_g else None

    A_all = np.stack(data["action"].values).astype(np.float64)
    S_all = np.stack(data["observation.state"].values).astype(np.float64)
    ep_col = data["episode_index"].values

    diffs = video_framediffs(root, info, epm, cams) if args.with_video else None

    per_ep: "OrderedDict[int, dict]" = OrderedDict()
    steady_means: dict[int, np.ndarray] = {}
    live_min: dict[int, np.ndarray] = {}
    live_max: dict[int, np.ndarray] = {}

    for _, r in epm.iterrows():
        e = int(r["episode_index"])
        m = np.where(ep_col == e)[0]
        n_meta = int(r["length"])
        A = A_all[m]
        S = S_all[m]
        n = len(m)
        res: dict = {"ep": e, "session": ep_run.get(e, {}).get("session", "?"), "n": n, "dur_s": n / fps}

        # ---- A 完全性
        fi = data["frame_index"].values[m]
        ts = data["timestamp"].values[m].astype(np.float64)
        res["len_ok"] = (n == n_meta) and (int(r["dataset_to_index"]) - int(r["dataset_from_index"]) == n_meta)
        res["fi_ok"] = bool(n > 0 and (fi == np.arange(n)).all())
        res["ts_err"] = float(np.abs(ts - fi / fps).max()) if n else float("nan")
        res["nan"] = int(np.isnan(A).sum() + np.isnan(S).sum() + np.isinf(A).sum() + np.isinf(S).sum())
        vid_ok = True
        for cam in cams:
            key = f"observation.images.{cam}"
            span = float(r[f"videos/{key}/to_timestamp"]) - float(r[f"videos/{key}/from_timestamp"])
            if abs(span - n_meta / fps) > 1.5 / fps:
                vid_ok = False
        res["vid_span_ok"] = vid_ok
        res["A_grade"] = "pass" if (res["len_ok"] and res["fi_ok"] and res["ts_err"] < 1e-3 and res["nan"] == 0 and vid_ok) else "fail"

        # ---- B カメラ / ループ（console）
        con = ep_run.get(e, {}).get("console", {}).get(e, {})
        res["takes"] = con.get("takes")
        res["stale"] = con.get("stale")
        res["slow_cycles"] = con.get("slow_cycles")
        res["min_hz"] = con.get("min_hz") if con.get("min_hz") not in (None, float("inf")) else None
        res["drops"] = con.get("drops")
        res["ob_ratio"] = (con["slow_cycles"] / n) if con and n else None
        # 壁時計の尺（参考値）: console.log の Recording 行から、終了キーの直前に出たログ行の時刻まで。
        # ログ行は数秒〜10 秒おきなので粗い。timestamp 列は frame_index/fps の合成値で実時間を持たないため、
        # 大量のフレーム落ち（壁時計 >> frames/fps）を見る唯一の手がかり。
        res["wall_s"] = None
        if con.get("start_ts") and con.get("end_ts"):
            from datetime import datetime
            try:
                t0 = datetime.strptime(con["start_ts"], "%Y-%m-%d %H:%M:%S") if len(con["start_ts"]) > 8 else None
                t1 = datetime.strptime(con["end_ts"], "%Y-%m-%d %H:%M:%S")
                if t0 is None:
                    t0 = datetime.combine(t1.date(), datetime.strptime(con["start_ts"], "%H:%M:%S").time())
                res["wall_s"] = (t1 - t0).total_seconds()
            except ValueError:
                pass
        g_stale = "n/a" if res["stale"] is None else ("pass" if res["stale"] == 0 else ("flag" if res["stale"] <= 0.01 * n else "fail"))
        g_ob = grade(res["ob_ratio"], args.tol_overbudget[0], args.tol_overbudget[1]) if res["ob_ratio"] is not None else "n/a"
        g_hz = "n/a" if res["min_hz"] is None else ("pass" if res["min_hz"] >= args.tol_minhz else "flag")
        g_drops = "n/a" if res["drops"] is None else ("pass" if res["drops"] == 0 else "fail")
        # 動画
        res["dup_mov"] = {}
        res["dup_all"] = {}
        res["dup_run"] = {}
        g_dup = "skip"
        if diffs is not None:
            sp = np.r_[0.0, np.abs(np.diff(S[:, idx_live_j], axis=0)).max(1) * fps]
            moving = sp > args.moving_speed
            for cam in cams:
                key = f"observation.images.{cam}"
                k = f"{cam}/{int(r[f'videos/{key}/chunk_index'])}/{int(r[f'videos/{key}/file_index'])}"
                if k not in diffs:
                    continue
                f0 = int(round(float(r[f"videos/{key}/from_timestamp"]) * fps))
                d = diffs[k][f0:f0 + n]
                if len(d) < n:
                    res["vid_frames_short"] = True
                dup = d < args.dup_diff
                res["dup_all"][cam] = float(np.nanmean(dup))
                res["dup_mov"][cam] = float(np.nanmean(dup[moving[: len(dup)]])) if moving[: len(dup)].any() else float("nan")
                res["dup_run"][cam] = int(longest_run(dup & moving[: len(dup)]))
            # 判定は「腕に載っていて画が必ず動く」wrist_<live> を主、無ければ全カメラの最大
            live_cam = f"wrist_{live_side}"
            v = res["dup_mov"].get(live_cam, max(res["dup_mov"].values()) if res["dup_mov"] else None)
            g_dup = grade(v, args.tol_dup[0], args.tol_dup[1]) if v is not None else "skip"
            runmax = max(res["dup_run"].values()) if res["dup_run"] else 0
            g_run = "pass" if runmax < args.tol_dup_run[0] else ("flag" if runmax < args.tol_dup_run[1] else "fail")
            g_dup = worst(g_dup, g_run)
        res["B_grade"] = worst(g_stale, g_ob, g_hz, g_drops, g_dup)
        res["B_detail"] = dict(stale=g_stale, overbudget=g_ob, minhz=g_hz, drops=g_drops, dup=g_dup)

        # ---- C 開始条件
        run = ep_run.get(e)
        entry = None
        if run:
            for x in run["start_entries"]:
                if int(x["episode_index"]) == e and str(x.get("outcome", "")).startswith("started"):
                    entry = x  # 最後の started* を採る
        res["start_outcome"] = entry.get("outcome") if entry else None
        res["start_worst_joint"] = entry.get("worst_joint") if entry else None
        res["start_worst_deg"] = float(entry.get("worst_deviation_deg")) if entry else None
        res["retakes"] = (con.get("takes", 1) - 1) if con else None
        ref = run["reference"] if run else None
        res["f0_worst_joint"] = None
        res["f0_worst_deg"] = None
        res["f0_live_dev"] = None
        if ref and n:
            dev = {}
            for i, nm in enumerate(names):
                if nm in ref and "gripper" not in nm and nm.startswith(live_side):
                    dev[nm] = abs(S[0, i] - float(ref[nm]))
            if dev:
                wj = max(dev, key=dev.get)
                res["f0_worst_joint"] = wj
                res["f0_worst_deg"] = float(dev[wj])
                res["f0_live_dev"] = {k.replace(".pos", ""): round(v, 2) for k, v in dev.items()}
        # jsonl の worst は右腕を含むが、右腕の参照は固定指令前の整列姿勢なので意味を持たない → 左腕だけで判定
        live_dev_jsonl = None
        if entry:
            dd = {k: v for k, v in entry.get("deviation_deg", {}).items() if k.startswith(live_side)}
            if dd:
                live_dev_jsonl = max(dd.values())
        res["start_live_deg"] = live_dev_jsonl
        c_val = max([v for v in (live_dev_jsonl, res["f0_worst_deg"]) if v is not None], default=None)
        res["C_grade"] = grade(c_val, args.tol_start_pass, args.tol_start_flag) if c_val is not None else "n/a"
        if res["start_outcome"] == "started_unhomed" and res["C_grade"] == "pass":
            res["C_grade"] = "flag"

        # ---- D 右腕（凍結側）
        Af = A[:, idx_frozen]
        Sf = S[:, idx_frozen]
        res["frozen_action_range"] = float((Af.max(0) - Af.min(0)).max()) if n else float("nan")
        # 定常 = 後半 50% の平均。過渡 = 先頭から、定常±tol を外れている最後のフレーム+1
        half = Sf[n // 2 :] if n > 1 else Sf
        mean_steady = half.mean(0)
        steady_means[e] = mean_steady
        tol = np.array([args.settle_grip_tol if "gripper" in names[i] else args.settle_tol for i in idx_frozen])
        off = np.abs(Sf - mean_steady) > tol
        bad_idx = np.where(off.any(1))[0]
        lead = bad_idx[bad_idx < n // 2]
        res["settle_frames"] = int(lead.max() + 1) if len(lead) else 0
        # 定常レンジは後半 50% で測る（先頭の過渡とその跳ねの尾を含めない。保持そのもののぶれを見る）
        rest = half
        res["frozen_state_range"] = float((rest.max(0) - rest.min(0)).max()) if len(rest) else float("nan")
        res["frozen_state_range_j"] = {names[i].replace(".pos", ""): round(float(v), 2) for i, v in zip(idx_frozen, rest.max(0) - rest.min(0))} if len(rest) else {}
        # 過渡の大きさ（先頭フレームの定常からのずれ・関節とグリッパの最大）
        res["frozen_f0_dev_max_joint"] = float(max(abs(Sf[0, k] - mean_steady[k]) for k, i in enumerate(idx_frozen) if "gripper" not in names[i]))
        res["frozen_f0_dev_grip"] = float(next(abs(Sf[0, k] - mean_steady[k]) for k, i in enumerate(idx_frozen) if "gripper" in names[i]))
        res["frozen_f0_dev"] = {names[i].replace(".pos", ""): round(float(v), 1) for i, v in zip(idx_frozen, np.abs(Sf[0] - mean_steady))}
        g_act = "pass" if res["frozen_action_range"] == 0 else "fail"
        g_rng = grade(res["frozen_state_range"], args.tol_right_range[0], args.tol_right_range[1])
        g_set = "pass" if res["settle_frames"] <= args.tol_right_settle[0] else ("flag" if res["settle_frames"] <= args.tol_right_settle[1] else "fail")
        res["D_grade_partial"] = worst(g_act, g_rng, g_set)
        res["D_detail"] = dict(action=g_act, range=g_rng, settle=g_set)

        # ---- E 左腕（実演側）
        Aj = A[:, idx_live_j]
        Sj = S[:, idx_live_j]
        err = np.abs(Aj - Sj)
        res["track_p95"] = float(np.percentile(err, 95, axis=0).max()) if n else float("nan")
        res["track_p95_j"] = [round(float(v), 2) for v in np.percentile(err, 95, axis=0)] if n else []
        res["track_max"] = float(err.max()) if n else float("nan")
        res["track_max_joint"] = names[idx_live_j[int(err.max(0).argmax())]].replace(".pos", "") if n else None
        if idx_live_g is not None:
            g = S[:, idx_live_g]
            thr = args.grip_threshold
            res["grip_open"] = int(((g[:-1] > thr) & (g[1:] <= thr)).sum())
            res["grip_close"] = int(((g[:-1] < thr) & (g[1:] >= thr)).sum())
            res["grip_open_frac"] = float((g < thr).mean())
            res["grip_min"] = float(g.min())
            res["grip_max"] = float(g.max())
            # 保持区間: close イベントから次の open イベントまでのうち、state の中央値が積み木幅の範囲にあるもの
            ev = []
            for i in range(1, n):
                if g[i - 1] > thr and g[i] <= thr:
                    ev.append((i, "open"))
                elif g[i - 1] < thr and g[i] >= thr:
                    ev.append((i, "close"))
            holds = []
            for (i, k), (j, _) in zip(ev, ev[1:] + [(n, None)]):
                if k == "close" and (j - i) >= args.grip_hold_min_s * fps:
                    med = float(np.median(g[i:j]))
                    if args.grip_hold_range[0] <= med <= args.grip_hold_range[1]:
                        holds.append(round(med, 1))
            res["grip_holds"] = len(holds)
            res["grip_hold_widths"] = holds
        else:
            res["grip_open"] = res["grip_close"] = res["grip_holds"] = None
            res["grip_hold_widths"] = []
        vel = np.abs(np.diff(Sj, axis=0)) * fps if n > 1 else np.zeros((1, len(idx_live_j)))
        sp = vel.max(1)
        res["speed_max"] = float(sp.max())
        res["speed_p99"] = float(np.percentile(sp, 99))
        res["speed_max_joint"] = names[idx_live_j[int(vel.max(0).argmax())]].replace(".pos", "")
        va = np.abs(np.diff(Aj, axis=0)) * fps if n > 1 else np.zeros((1, len(idx_live_j)))
        res["action_jump_max"] = float(va.max())
        idle = sp < 2.0
        res["idle_frac"] = float(idle.mean())
        res["idle_longest_s"] = longest_run(idle) / fps
        g_p95 = grade(res["track_p95"], args.tol_track_p95[0], args.tol_track_p95[1])
        g_max = grade(res["track_max"], args.tol_track_max[0], args.tol_track_max[1])
        g_grip = "n/a" if res["grip_holds"] is None else ("pass" if res["grip_holds"] >= args.min_grip_holds else "fail")
        g_spd = grade(res["speed_max"], args.tol_speed[0], args.tol_speed[1])
        g_jump = "pass" if res["action_jump_max"] <= args.tol_speed[1] else "fail"
        g_idle = "pass" if res["idle_longest_s"] <= args.tol_idle else "flag"
        res["E_grade"] = worst(g_p95, g_max, g_grip, g_spd, g_jump, g_idle)
        res["E_detail"] = dict(p95=g_p95, max=g_max, grip=g_grip, speed=g_spd, jump=g_jump, idle=g_idle)

        live_min[e] = Sj.min(0)
        live_max[e] = Sj.max(0)
        per_ep[e] = res

    # ---- D の続き: エピソード間の定常姿勢ずれ（右腕）
    if steady_means:
        M = np.stack([steady_means[e] for e in per_ep])
        ref_mean = np.median(M, axis=0)
        for e, res in per_ep.items():
            dv = np.abs(steady_means[e] - ref_mean)
            # gripper は 5° まで許す（開放保持の遊び）
            scale = np.array([0.2 if "gripper" in names[i] else 1.0 for i in idx_frozen])
            res["frozen_mean_dev"] = float((dv * scale).max())
            g_mean = grade(res["frozen_mean_dev"], args.tol_right_mean[0], args.tol_right_mean[1])
            res["D_detail"]["mean"] = g_mean
            res["D_grade"] = worst(res["D_grade_partial"], g_mean)

    # ---- F 分布（baseline との比較）
    base_len = np.array([per_ep[e]["n"] for e in baseline if e in per_ep]) if baseline else np.array([])
    base_med = float(np.median(base_len)) if len(base_len) else float("nan")
    if baseline:
        env_min = np.min(np.stack([live_min[e] for e in baseline if e in live_min]), axis=0)
        env_max = np.max(np.stack([live_max[e] for e in baseline if e in live_max]), axis=0)
    for e, res in per_ep.items():
        res["len_ratio"] = res["n"] / base_med if base_med and not math.isnan(base_med) else float("nan")
        run = ep_run.get(e)
        cap = run.get("episode_time_s") if run else None
        res["hit_cap"] = bool(cap and res["n"] >= cap * fps - 2)
        g_len = "pass" if (math.isnan(res["len_ratio"]) or res["len_ratio"] <= args.tol_len_ratio) else "flag"
        if res["hit_cap"]:
            g_len = "flag"
        if baseline:
            ex_lo = np.maximum(env_min - live_min[e], 0)
            ex_hi = np.maximum(live_max[e] - env_max, 0)
            ex = np.maximum(ex_lo, ex_hi)
            res["range_exceed"] = float(ex.max())
            res["range_exceed_joint"] = names[idx_live_j[int(ex.argmax())]].replace(".pos", "") if ex.max() > 0 else "-"
        else:
            res["range_exceed"] = float("nan")
        g_rng = grade(res["range_exceed"], args.tol_range_exceed[0], args.tol_range_exceed[1]) if not math.isnan(res["range_exceed"]) else "n/a"
        if e in baseline:
            g_rng = "pass" if g_rng != "n/a" else g_rng  # baseline 自身は包絡の定義側
        res["F_grade"] = worst(g_len, g_rng)
        res["F_detail"] = dict(len=g_len, range=g_rng)
        res["grade"] = worst(res["A_grade"], res["B_grade"], res["C_grade"], res.get("D_grade", res["D_grade_partial"]), res["E_grade"], res["F_grade"])
        reasons = []
        for tag, det in (("B", res["B_detail"]), ("C", {"start": res["C_grade"]}), ("D", res["D_detail"]), ("E", res["E_detail"]), ("F", res["F_detail"])):
            for k, v in det.items():
                if v in ("flag", "fail"):
                    reasons.append(f"{tag}:{k}={v}")
        if res["A_grade"] == "fail":
            reasons.insert(0, "A:fail")
        res["reasons"] = ", ".join(reasons) if reasons else "-"

    # ---- 系統的所見: 半数以上のエピソードに共通する理由はエピソード単位の採否から分ける
    reason_keys: dict[str, int] = {}
    for res in per_ep.values():
        for tag, det in (("B", res["B_detail"]), ("C", {"start": res["C_grade"]}), ("D", res["D_detail"]), ("E", res["E_detail"]), ("F", res["F_detail"])):
            for k, v in det.items():
                if v in ("flag", "fail"):
                    reason_keys[f"{tag}:{k}"] = reason_keys.get(f"{tag}:{k}", 0) + 1
    n_ep = max(len(per_ep), 1)
    systematic = {k: c for k, c in reason_keys.items() if c / n_ep >= args.systematic_frac}
    for res in per_ep.values():
        g = "pass"
        ex_reasons = []
        for tag, det in (("B", res["B_detail"]), ("C", {"start": res["C_grade"]}), ("D", res["D_detail"]), ("E", res["E_detail"]), ("F", res["F_detail"])):
            for k, v in det.items():
                if v in ("flag", "fail") and f"{tag}:{k}" not in systematic:
                    g = worst(g, v)
                    ex_reasons.append(f"{tag}:{k}={v}")
        if res["A_grade"] == "fail":
            g = "fail"
            ex_reasons.insert(0, "A:fail")
        res["grade_ex"] = g
        res["reasons_ex"] = ", ".join(ex_reasons) if ex_reasons else "-"

    # ------------------------------------------------------------------ 出力
    print(f"# 受入検査（案） dataset={root.name}  episodes={ep_list[0]}..{ep_list[-1]} ({len(ep_list)} 本)  baseline={baseline[0]}..{baseline[-1]} ({len(baseline)} 本)\n")
    print("## run の整合（統制変数が run 間でずれていたら混ぜられない）\n")
    rows = []
    for run in runs:
        rows.append([
            run["session"], run["name"], run.get("started_local", "-"), run.get("operator", "-"), fmt(run.get("episode_time_s")),
            run.get("freeze_side") or "-", run.get("frozen_gripper_deg") or "-", run.get("gripper_kp") or "-", run.get("scene_sha", "-"),
            ",".join(f"{k}={v}" for k, v in sorted((run.get("calib_sha") or {}).items())),
            ",".join(f"{k}={v}" for k, v in sorted((run.get("cameras") or {}).items())), fmt(run.get("exit_code")),
            fmt(run["reference"].get(f"{live_side}_joint_1.pos") if run.get("reference") else None, 2),
        ])
    table(["session", "run", "started", "operator", "EPISODE_TIME_S", "FREEZE_SIDE", "FROZEN_GRIP", "GRIPPER_KP", "scene sha", "calib sha", "cameras", "exit", f"ref {live_side} J1"], rows)
    scene_set = {r.get("scene_sha") for r in runs}
    calib_set = {json.dumps(r.get("calib_sha"), sort_keys=True) for r in runs}
    cam_set = {json.dumps(r.get("cameras"), sort_keys=True) for r in runs}
    print(f"- scene sha 一致: {'yes' if len(scene_set) <= 1 else 'NO'} / 較正 sha 一致: {'yes' if len(calib_set) <= 1 else 'NO'} / カメラ serial 一致: {'yes' if len(cam_set) <= 1 else 'NO'}")
    refs = [r["reference"] for r in runs if r.get("reference")]
    if len(refs) >= 2:
        keys = [k for k in refs[0] if "gripper" not in k]
        spread = {k.replace(".pos", ""): round(max(float(x[k]) for x in refs) - min(float(x[k]) for x in refs), 2) for k in keys}
        big = {k: v for k, v in spread.items() if v >= 1.0}
        print(f"- run 間の開始姿勢参照のばらつき（max-min, deg）: {big if big else '全関節 1° 未満'}")
    print()

    print("## エピソード表（値と判定案）\n")
    hdr = ["ep", "sess", "frames", "s", "wall s", "A", "stale", "ob cyc", "min Hz", "→ live dev", "f0 dev", "outcome", "retake", "R act rng", "R settle fr", "R f0 dev J/G", "R rng", "R mean dev", "L p95", "L max", "grip o/c/hold", "spd max", "idle s", "len ratio", "rng exceed"]
    if diffs is not None:
        hdr += ["dup mov(chest/wr/wl)", "dup run"]
    hdr += ["grade(全)", "reasons(全)", "grade(系統除く)", "reasons(系統除く)"]
    rows = []
    for e, res in per_ep.items():
        row = [e, res["session"], res["n"], fmt(res["dur_s"]), fmt(res["wall_s"], 0), res["A_grade"], fmt(res["stale"]), fmt(res["slow_cycles"]), fmt(res["min_hz"]),
               fmt(res["start_live_deg"]), fmt(res["f0_worst_deg"]), (res["start_outcome"] or "-").replace("started_", ""), fmt(res["retakes"]),
               fmt(res["frozen_action_range"], 2), res["settle_frames"], f"{fmt(res['frozen_f0_dev_max_joint'])}/{fmt(res['frozen_f0_dev_grip'], 0)}", fmt(res["frozen_state_range"], 2), fmt(res.get("frozen_mean_dev"), 2),
               fmt(res["track_p95"]), fmt(res["track_max"]), f"{fmt(res['grip_open'])}/{fmt(res['grip_close'])}/{fmt(res['grip_holds'])}", fmt(res["speed_max"], 0), fmt(res["idle_longest_s"]),
               fmt(res["len_ratio"], 2), f"{fmt(res['range_exceed'])} ({res.get('range_exceed_joint', '-')})"]
        if diffs is not None:
            row += ["/".join(f"{res['dup_mov'].get(c, float('nan'))*100:.1f}%" for c in cams), "/".join(str(res["dup_run"].get(c, "-")) for c in cams)]
        row += [res["grade"], res["reasons"], res["grade_ex"], res["reasons_ex"]]
        rows.append(row)
    table(hdr, rows)

    print("## 系統的所見（半数以上のエピソードに共通。エピソード単位の採否ではなく手順・コードの修正対象）\n")
    if systematic:
        table(["理由", "該当本数", "割合"], [[k, c, f"{c / n_ep:.0%}"] for k, c in sorted(systematic.items())])
    else:
        print("- なし\n")

    print("## セッション集計\n")
    rows = []
    for sess in sorted({r["session"] for r in per_ep.values()}, key=lambda s: (len(s), s)):
        sub = [r for r in per_ep.values() if r["session"] == sess]
        cnt = {g: sum(1 for r in sub if r["grade"] == g) for g in ("pass", "flag", "fail")}
        cnt_ex = {g: sum(1 for r in sub if r["grade_ex"] == g) for g in ("pass", "flag", "fail")}
        lens = np.array([r["n"] for r in sub])
        rows.append([sess, len(sub), f"{cnt['pass']}/{cnt['flag']}/{cnt['fail']}", f"{cnt_ex['pass']}/{cnt_ex['flag']}/{cnt_ex['fail']}",
                     f"{lens.min()}-{lens.max()} (med {int(np.median(lens))})",
                     sum(r["stale"] or 0 for r in sub), sum(r["slow_cycles"] or 0 for r in sub),
                     sum(1 for r in sub if (r["start_live_deg"] or 0) > args.tol_start_pass), sum(1 for r in sub if r["settle_frames"] > 0),
                     fmt(float(np.median([r["track_p95"] for r in sub])), 2), fmt(float(np.median([r["frozen_state_range"] for r in sub])), 2),
                     ", ".join(f"ep{r['ep']}({r['reasons_ex']})" for r in sub if r["grade_ex"] != "pass") or "-"])
    table(["session", "本数", "pass/flag/fail(全)", "pass/flag/fail(系統除く)", "frames", "stale", "ob cyc", f"開始ずれ>{args.tol_start_pass:g}°", "右腕過渡あり", "L p95 med", "R rng med", "系統除きで pass 以外"], rows)

    print("## 閾値（案・既定値）\n")
    rows = [
        ["C 開始ずれ（→ 時点・先頭フレーム、左腕 worst）", f"≤{args.tol_start_pass:g}° pass / ≤{args.tol_start_flag:g}° flag / それ以上 fail。started_unhomed は最低 flag"],
        ["D 右腕 action レンジ", "0 のみ pass（固定指令は完全一定であるべき）"],
        ["D 右腕 state 定常レンジ（後半 50%）", f"≤{args.tol_right_range[0]:g}° pass / ≤{args.tol_right_range[1]:g}° flag"],
        ["D 右腕 先頭過渡フレーム数", f"≤{args.tol_right_settle[0]} pass / ≤{args.tol_right_settle[1]} flag"],
        ["D 右腕 エピソード間定常ずれ", f"≤{args.tol_right_mean[0]:g}° pass / ≤{args.tol_right_mean[1]:g}° flag（gripper は 1/5 重み）"],
        ["E 左腕追従 p95 / max", f"≤{args.tol_track_p95[0]:g}/≤{args.tol_track_max[0]:g}° pass、≤{args.tol_track_p95[1]:g}/≤{args.tol_track_max[1]:g}° flag"],
        ["E gripper 保持回数", f"物を保持した区間（state {args.grip_hold_range[0]:g}..{args.grip_hold_range[1]:g}° が {args.grip_hold_min_s:g} s 以上）が ≥{args.min_grip_holds} でなければ fail。open/close は参考"],
        ["E 左腕速度 max / action 跳び", f"≤{args.tol_speed[0]:g} deg/s pass / ≤{args.tol_speed[1]:g} flag / 超えたら fail（action 跳び >{args.tol_speed[1]:g} も fail）"],
        ["E 最長静止", f">{args.tol_idle:g} s で flag"],
        ["B stale frame / CAN drops", "0 のみ pass（stale は ≤1% flag・超えたら fail、drops は 1 行でも fail）"],
        ["B fps 超過（遅れサイクル/フレーム）", f"≤{args.tol_overbudget[0]:.0%} pass / ≤{args.tol_overbudget[1]:.0%} flag、最遅 <{args.tol_minhz:g} Hz で flag"],
        ["B 動画重複（左腕が動く区間・wrist_live）", f"≤{args.tol_dup[0]:.0%} pass / ≤{args.tol_dup[1]:.0%} flag、連続 ≥{args.tol_dup_run[0]} で flag / ≥{args.tol_dup_run[1]} で fail"],
        ["F 長さ / 可動範囲", f"長さ >{args.tol_len_ratio:g}× baseline 中央値 または上限到達で flag、包絡超え ≤{args.tol_range_exceed[0]:g}° pass / ≤{args.tol_range_exceed[1]:g}° flag"],
    ]
    table(["項目", "案"], rows)

    if args.verbose:
        print("## 詳細（関節別）\n")
        rows = []
        for e, res in per_ep.items():
            rows.append([e, json.dumps(res.get("f0_live_dev")), json.dumps(res["frozen_f0_dev"]), json.dumps(res["frozen_state_range_j"]), json.dumps(res["track_p95_j"]), res["track_max_joint"], res["speed_max_joint"], json.dumps(res["grip_hold_widths"])])
        table(["ep", "f0 live dev", "R f0 dev from steady", "R steady range", "L p95 by joint", "L max joint", "spd max joint", "grip hold widths"], rows)

    if args.json:
        def _conv(o):
            if isinstance(o, (np.floating, np.integer)):
                return o.item()
            if isinstance(o, np.ndarray):
                return o.tolist()
            return str(o)
        Path(args.json).write_text(json.dumps({"episodes": list(per_ep.values()), "baseline": baseline, "runs": [{k: v for k, v in r.items() if k not in ("console", "start_entries")} for r in runs]}, ensure_ascii=False, indent=1, default=_conv))
        print(f"[accept] wrote {args.json}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
