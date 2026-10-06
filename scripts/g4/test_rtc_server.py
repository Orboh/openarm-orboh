#!/usr/bin/env python3
"""
test_rtc_server.py — gr00t_rtc_server.py の検証（実機・CAN に触らない。推論サーバに観測を送るだけ・2026-10-07）

試走の記録（traj.csv の 1 行の関節角と、同じ step の 3 眼の画像）を観測にして、次を確かめる。
  1. options なしの推論が従来どおり返る（チャンク長・推論時間）。
  2. RTC・frozen = overlap: 返ったチャンクの冒頭 overlap step が、送った前のチャンクと一致する。
     frozen の step はモデルが更新しないので、一致すれば「物理値の絶対角 → 今の状態基準の相対値 → 正規化」の
     変換と、その逆（decode_action）の往復が正しい。ずれは bf16 と正規化の丸めの分だけのはず。
  3. RTC・frozen = 0: 冒頭が前のチャンクに寄る（options なしの 2 回の差より小さい）。
  4. options なしで 2 回推論したときの差（RTC が無いときの「つなぎ目のずれ」の目安）。

実行（Thor・lerobot の venv。サーバは rollout_bi_takeei.sh server-rtc で起動しておく）:
    ~/projects/lerobot/.venv/bin/python scripts/g4/test_rtc_server.py --rollout-dir ~/rollouts/20261006T140530Z-bi-takeei-r1-03
"""

import argparse
import csv
import glob
import os
import sys
import time

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import eval_openarm as ev  # noqa: E402

LANG = "Clear the garbage bag, the 2x4 lumber and the white bar off the blue sheet, then remove the blue sheet from the table"
CAMS = ["chest", "wrist_right", "wrist_left"]


def load_obs(rollout_dir: str, row_index: int) -> dict:
    trajs = glob.glob(os.path.join(rollout_dir, "**", "traj.csv"), recursive=True)
    if not trajs:
        raise SystemExit(f"traj.csv が無い: {rollout_dir}")
    rows = list(csv.DictReader(open(trajs[0])))
    row = rows[row_index]
    step = int(row["step"])
    base = os.path.dirname(trajs[0])
    obs = {k: float(row[f"present.{k}"]) for k in ev.BIMANUAL_STATE_KEYS}
    chest = sorted(glob.glob(os.path.join(base, "frames", "frame_*.npy")))
    obs["chest"] = np.load(min(chest, key=lambda p: abs(int(p[-9:-4]) - step))) if chest else np.zeros((480, 640, 3), np.uint8)
    for cam in CAMS[1:]:
        jpgs = sorted(glob.glob(os.path.join(base, f"frames_{cam}", "frame_*.jpg")))
        if jpgs:
            from PIL import Image
            p = min(jpgs, key=lambda q: abs(int(q[-9:-4]) - step))
            obs[cam] = np.asarray(Image.open(p).convert("RGB"))
        else:
            obs[cam] = np.zeros((480, 640, 3), np.uint8)
    obs["lang"] = LANG
    print(f"観測: {trajs[0]} の {row_index} 行目（step {step}）・画像 {[obs[c].shape for c in CAMS]}")
    return obs


def chunk_array(actions: list) -> np.ndarray:
    return np.array([[a[k] for k in ev.BIMANUAL_STATE_KEYS] for a in actions], dtype=np.float64)


def report(name: str, diff: np.ndarray) -> None:
    arm = [i for i, k in enumerate(ev.BIMANUAL_STATE_KEYS) if "gripper" not in k]
    grip = [i for i, k in enumerate(ev.BIMANUAL_STATE_KEYS) if "gripper" in k]
    print(f"  {name}: 腕の最大 {np.abs(diff[:, arm]).max():.3f} 度・中央値 {np.median(np.abs(diff[:, arm])):.3f}"
          f" / gripper の最大 {np.abs(diff[:, grip]).max():.3f}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--rollout-dir", required=True)
    ap.add_argument("--row", type=int, default=300)
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=5555)
    ap.add_argument("--offset", type=int, default=4, help="前のチャンクの何番目から送るか")
    ap.add_argument("--overlap", type=int, default=8)
    ap.add_argument("--ramp", type=float, nargs="+", default=[5.0], help="frozen=0 の試験で使う ramp_rate（複数可）")
    a = ap.parse_args()

    client = ev.PolicyClient(host=a.host, port=a.port, timeout_ms=60000)
    if not client.ping():
        raise SystemExit("PolicyServer に届かない。rollout_bi_takeei.sh server-rtc で起動しておく")
    pol = ev.OpenArmAdapter(client, bimanual=True, camera_keys=CAMS)
    obs = load_obs(a.rollout_dir, a.row)

    def infer(options=None):
        t0 = time.time()
        actions, raw, info = pol.get_chunk(obs, options)
        return actions, raw, info, (time.time() - t0) * 1000

    infer()  # 1 回目は暖機
    A, rawA, _, msA = infer()
    H = len(A)
    print(f"[1] options なし: チャンク長 {H}・推論 {msA:.0f}ms")
    o, n = a.offset, min(a.overlap, H - a.offset)
    prev = {k: np.ascontiguousarray(v[:, o:o + n, :], dtype=np.float32) for k, v in rawA.items()}
    target = chunk_array(A)[o:o + n]

    B, _, infoB, msB = infer({"rtc": {"prev_action": prev, "frozen": n, "ramp_rate": 5.0}})
    print(f"[2] RTC frozen=overlap={n}（A の {o}〜{o + n - 1} 番目を冒頭に固定）: 推論 {msB:.0f}ms・info {infoB}")
    report("B の冒頭 - 送った前のチャンク（0 に近いほど変換の往復が正しい）", chunk_array(B)[:n] - target)
    print(f"  B の {n} 番目と {n - 1} 番目の差（固定の後のつなぎ目）: "
          f"{np.abs(chunk_array(B)[n] - chunk_array(B)[n - 1]).max():.3f}" if n < H else "")

    for ramp in a.ramp:
        C, _, _, msC = infer({"rtc": {"prev_action": prev, "frozen": 0, "ramp_rate": ramp}})
        print(f"[3] RTC frozen=0・ramp_rate {ramp}（ramp だけで寄せる）: 推論 {msC:.0f}ms")
        report("C の冒頭 - 送った前のチャンク", chunk_array(C)[:n] - target)

    D, _, _, msD = infer()
    print(f"[4] options なしをもう 1 回: 推論 {msD:.0f}ms")
    report("D の冒頭 - 送った前のチャンク（RTC なしの基準）", chunk_array(D)[o:o + n] - target)
    report("D - A（同じ観測で 2 回推論した差・ノイズの分）", chunk_array(D) - chunk_array(A))


if __name__ == "__main__":
    main()
