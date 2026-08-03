#!/usr/bin/env python
"""
G4 ロールアウトログ → LeRobotDataset 変換（オフライン）

eval_openarm.py が `--trajectory_log`（CSV）と `--frame_log_dir`（npy）に落とした
自律実行の記録を、収集データセットと同じスキーマの LeRobotDataset に変換する。

**実機の実行経路には触れない。** 本スクリプトは記録済みファイルを読むだけで、
ロボット・PolicyServer のいずれにも接続しない。

前提と設計上の注意（2026-07-22 実測にもとづく）:

  eval_openarm.py は `robot.get_observation()` を **chunk の先頭でしか呼ばない**。
  action_horizon=8・fps=10 の実行では、CSV 150 行のうち 131 行の `present.*` は
  前チャンクの値の使い回し（実測で 131/131 が完全一致）。画像も同じ地点でしか
  取れないため、記録側を直さない限り **観測は chunk レート（実測 約1Hz）が上限**になる。

  そのため本スクリプトは観測粒度を自動判定する:
    - chunk 内で `present.*` が変化していれば → 全行を採用（記録側修正済み）
    - 変化していなければ        → chunk 先頭行のみ採用（観測が実在する行だけ）
  判定結果は README とログに必ず残す。粒度を偽らないことを優先する。

使い方:

    ~/projects/lerobot/.venv/bin/python convert_trajlog_to_lerobot.py \
        --trajectory-log ~/g4-artifacts/results/traj_aligned.csv \
        --frame-log-dir  ~/g4-artifacts/frames/run_20260722_1413 \
        --repo-id Orboh/openarm_bottle_pnp_rollouts_v1 \
        --task "pick up the bottle and place it" \
        --model-repo Orboh/gr00t-openarm-bottle-pnp-chest-v1

  複数の実行をまとめて1データセットにする場合は --trajectory-log を複数渡す
  （CSV 1本 = 1エピソード。--frame-log-dir も同じ数だけ、同じ順で渡す）。

  HF への push は既定で行わない。`--push-to-hub` を明示した時だけ行う（private 固定）。
"""

from __future__ import annotations

import argparse
import csv
import json
import logging
import shutil
import statistics
import sys
from pathlib import Path

import numpy as np

# 収集データセット（Orboh/openarm_bottle_pnp_chest_v1）と一致させる。
# ここを勝手に変えると rollout と収集データの突き合わせが壊れる。
JOINT_NAMES = [
    "joint_1.pos",
    "joint_2.pos",
    "joint_3.pos",
    "joint_4.pos",
    "joint_5.pos",
    "joint_6.pos",
    "joint_7.pos",
    "gripper.pos",
]
CAMERA_KEY = "observation.images.chest"
IMAGE_SHAPE = (480, 640, 3)
ROBOT_TYPE = "openarm_follower"

# 記録間隔から fps を推定する際の許容範囲。外れたら黙って丸めず中止する
# （動画エンコーダが極端な fps を受け付けない・実行レートを偽らないため）。
FPS_MIN, FPS_MAX = 1, 240


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(
        description="G4 ロールアウトログを LeRobotDataset に変換する（オフライン・実機非接触）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    p.add_argument(
        "--trajectory-log",
        nargs="+",
        required=True,
        type=Path,
        help="eval_openarm.py --trajectory_log が出力した CSV（複数可・1本=1エピソード）",
    )
    p.add_argument(
        "--frame-log-dir",
        nargs="*",
        default=[],
        type=Path,
        help="eval_openarm.py --frame_log_dir（frame_XXXXX.npy）。--trajectory-log と同数・同順",
    )
    p.add_argument("--repo-id", required=True, help="出力データセット名（データ管理標準 §4 の `_rollouts` 命名）")
    p.add_argument("--task", required=True, help="実行時の言語指示（--lang_instruction と同じ文字列を入れる）")
    p.add_argument("--root", type=Path, default=None, help="ローカル出力先（既定は HF キャッシュ配下）")
    p.add_argument("--model-repo", default=None, help="このロールアウトを生成したモデル（README に記録）")
    p.add_argument("--notes", default=None, help="README に残す補足（実行条件・目視結果など）")
    p.add_argument(
        "--granularity",
        choices=["auto", "chunk", "step"],
        default="auto",
        help="観測粒度。auto は chunk 内で present.* が変化するかで判定する",
    )
    p.add_argument(
        "--fps",
        type=int,
        default=None,
        help="出力 fps。既定は採用行の中央値間隔から算出する",
    )
    p.add_argument(
        "--min-frame-coverage",
        type=float,
        default=0.9,
        help="採用行のうち対応する画像が存在する割合の下限。下回ったら中止する",
    )
    p.add_argument(
        "--allow-no-video",
        action="store_true",
        help="画像なしで変換する（状態・指令のみのデータセットになる）。既定は中止",
    )
    p.add_argument("--push-to-hub", action="store_true", help="HF に push する（private 固定）")
    p.add_argument("--overwrite", action="store_true", help="出力先が既にあれば消してから作る")
    p.add_argument("--dry-run", action="store_true", help="解析と検証だけ行い、データセットを書かない")
    return p.parse_args()


def read_csv(path: Path) -> list[dict]:
    with path.open(newline="") as f:
        rows = list(csv.DictReader(f))
    if not rows:
        raise SystemExit(f"{path} が空だ")
    required = {"step", "chunk_i", "t_sec"}
    required |= {f"present.{k}" for k in JOINT_NAMES}
    required |= {f"cmd.{k}" for k in JOINT_NAMES}
    missing = required - set(rows[0].keys())
    if missing:
        raise SystemExit(f"{path} に必要な列が無い: {sorted(missing)}")
    return rows


def present_vec(row: dict) -> tuple[str, ...]:
    return tuple(row[f"present.{k}"] for k in JOINT_NAMES)


def detect_granularity(rows: list[dict]) -> tuple[str, str]:
    """観測が chunk 内で更新されているかを実データから判定する。"""
    inner = [i for i, r in enumerate(rows) if int(r["chunk_i"]) != 0 and i > 0]
    if not inner:
        return "step", "chunk_i が常に 0（action_horizon=1 相当）→ 全行を採用する"
    stale = sum(1 for i in inner if present_vec(rows[i]) == present_vec(rows[i - 1]))
    ratio = stale / len(inner)
    if ratio > 0.5:
        return (
            "chunk",
            f"chunk 内の {stale}/{len(inner)} 行で present.* が前行と同一"
            "（観測は chunk 先頭でしか取られていない）→ chunk 先頭行のみ採用する",
        )
    return (
        "step",
        f"chunk 内の {stale}/{len(inner)} 行のみ前行と同一（観測は毎ステップ更新されている）→ 全行を採用する",
    )


def select_rows(rows: list[dict], granularity: str) -> list[dict]:
    if granularity == "step":
        return rows
    return [r for r in rows if int(r["chunk_i"]) == 0]


def load_frame_index(frame_dir: Path | None) -> dict[int, Path]:
    if frame_dir is None:
        return {}
    if not frame_dir.is_dir():
        raise SystemExit(f"--frame-log-dir が存在しない: {frame_dir}")
    index: dict[int, Path] = {}
    for f in sorted(frame_dir.glob("frame_*.npy")):
        try:
            index[int(f.stem.split("_")[1])] = f
        except (IndexError, ValueError):
            logging.warning(f"命名が想定外のため無視する: {f}")
    return index


def load_image(path: Path) -> np.ndarray:
    img = np.load(path, allow_pickle=False)
    if img.shape != IMAGE_SHAPE:
        raise SystemExit(
            f"{path} の shape が {img.shape}（期待 {IMAGE_SHAPE}）。"
            "収集データセットと解像度が違うと突き合わせができない"
        )
    if img.dtype != np.uint8:
        if np.issubdtype(img.dtype, np.floating) and float(img.max()) <= 1.0:
            img = (img * 255).round().astype(np.uint8)
        else:
            img = img.astype(np.uint8)
    return img


def infer_fps(sel: list[dict]) -> tuple[int, float]:
    t = [float(r["t_sec"]) for r in sel]
    dts = [t[i + 1] - t[i] for i in range(len(t) - 1) if t[i + 1] > t[i]]
    if not dts:
        return 1, 0.0
    med = statistics.median(dts)
    return max(1, round(1.0 / med)), med


def build_features(with_video: bool) -> dict:
    features = {
        "observation.state": {"dtype": "float32", "shape": (len(JOINT_NAMES),), "names": JOINT_NAMES},
        "action": {"dtype": "float32", "shape": (len(JOINT_NAMES),), "names": JOINT_NAMES},
        # 実測の経過秒。fps を丸めるため timestamp だけでは実時刻が失われる。
        # 「観測手段が無いものは後から検証できない」（2026-07-22 の教訓）ので生の時刻を残す。
        "t_sec": {"dtype": "float32", "shape": (1,), "names": None},
    }
    if with_video:
        features[CAMERA_KEY] = {
            "dtype": "video",
            "shape": IMAGE_SHAPE,
            "names": ["height", "width", "channels"],
        }
    return features


def write_readme(root: Path, meta: dict) -> None:
    lines = [
        f"# {meta['repo_id']}",
        "",
        "GR00T×OpenArm の**自律実行（ロールアウト）ログ**を LeRobotDataset 化したもの。",
        "teleop 実演デモではない（データ管理標準 §4 の `_rollouts` 種別）。",
        "",
        "## 派生元",
        "",
        "| 項目 | 値 |",
        "|---|---|",
        f"| 生成元スクリプト | `scripts/g4/convert_trajlog_to_lerobot.py` |",
        f"| 実行ブリッジ | `scripts/g4/eval_openarm.py` |",
        f"| モデル | {meta.get('model_repo') or '（未記録）'} |",
        f"| 言語指示 | {meta['task']} |",
        f"| 観測粒度 | {meta['granularity']} |",
        f"| fps | {meta['fps']}（採用行の中央値間隔 {meta['median_dt']:.3f}s から算出） |",
        f"| 画像 | {'あり' if meta['with_video'] else '**なし**（--allow-no-video で生成）'} |",
        "",
        "## 元ログ",
        "",
        "| エピソード | 軌道 CSV | フレーム | 元行数 | 採用 | 画像欠落で除外 |",
        "|---|---|---|---|---|---|",
    ]
    for e in meta["episodes"]:
        lines.append(
            f"| {e['index']} | `{e['csv']}` | `{e['frames'] or '—'}` | "
            f"{e['n_raw']} | {e['n_used']} | {e['n_dropped']} |"
        )
    lines += [
        "",
        "## 読むときの注意",
        "",
        meta["granularity_reason"],
        "",
        "`t_sec` は記録時の実経過秒。`timestamp` は fps を丸めた等間隔値なので、",
        "実行タイミングを議論するときは `t_sec` を使う。",
        "",
    ]
    if meta.get("notes"):
        lines += ["## 補足", "", meta["notes"], ""]
    (root / "README.md").write_text("\n".join(lines), encoding="utf-8")


def main() -> int:
    logging.basicConfig(level=logging.INFO, format="%(levelname)s %(message)s")
    args = parse_args()

    if args.frame_log_dir and len(args.frame_log_dir) != len(args.trajectory_log):
        raise SystemExit(
            f"--frame-log-dir は --trajectory-log と同数・同順で渡す"
            f"（{len(args.frame_log_dir)} vs {len(args.trajectory_log)}）"
        )

    episodes = []
    granularity_reason = ""
    for i, csv_path in enumerate(args.trajectory_log):
        rows = read_csv(csv_path)
        if args.granularity == "auto":
            gran, reason = detect_granularity(rows)
        else:
            gran, reason = args.granularity, f"--granularity {args.granularity} で明示指定された"
        granularity_reason = reason
        sel = select_rows(rows, gran)

        frame_dir = args.frame_log_dir[i] if args.frame_log_dir else None
        frames = load_frame_index(frame_dir)
        if frames:
            kept = [r for r in sel if int(r["step"]) in frames]
            coverage = len(kept) / len(sel)
        else:
            kept, coverage = sel, 0.0

        episodes.append(
            {
                "index": i,
                "csv": str(csv_path),
                "frames": str(frame_dir) if frame_dir else "",
                "n_raw": len(rows),
                "n_sel": len(sel),
                "n_used": len(kept),
                "n_dropped": len(sel) - len(kept),
                "granularity": gran,
                "coverage": coverage,
                "rows": kept,
                "frame_index": frames,
            }
        )
        logging.info(
            f"[ep{i}] {csv_path.name}: {len(rows)} 行 → 粒度 {gran} で {len(sel)} 行採用 / "
            f"画像 {len(frames)} 枚・被覆 {coverage:.1%} → 最終 {len(kept)} 行"
        )
        logging.info(f"[ep{i}] 判定根拠: {reason}")

    with_video = any(e["frame_index"] for e in episodes)
    if not with_video:
        if not args.allow_no_video:
            raise SystemExit(
                "画像が1枚も見つからない。ロールアウトデータセットは Foresight 等の検証に使うため、\n"
                "  画像なしでは用を成さない可能性が高い。\n"
                "  - 記録時に --frame_log_dir を指定していたか確認する\n"
                "  - 状態・指令だけで良いと判断したなら --allow-no-video を明示する"
            )
        logging.warning("画像なしで変換する（--allow-no-video）。状態・指令のみのデータセットになる")
    else:
        worst = min(e["coverage"] for e in episodes)
        if worst < args.min_frame_coverage:
            raise SystemExit(
                f"画像の被覆が {worst:.1%} で下限 {args.min_frame_coverage:.0%} を下回る。\n"
                "  eval_openarm.py の frame_log_every は **chunk 先頭でしか評価されない**ため、\n"
                "  action_horizon と互いに素でない値だと画像がほとんど残らない\n"
                "  （既定 horizon=8・every=25 では 200 step に1枚しか保存されない）。\n"
                "  記録側を直すか、--min-frame-coverage を下げて意図的に疎なまま作る。"
            )

    all_sel = [r for e in episodes for r in e["rows"]]
    if not all_sel:
        raise SystemExit("採用行が0だ")
    fps, median_dt = infer_fps(episodes[0]["rows"])
    if args.fps:
        fps = args.fps
    elif not (FPS_MIN <= fps <= FPS_MAX):
        # 記録間隔から算出した fps が現実的な範囲を外れている。
        # 動画エンコーダは極端な fps を受け付けない（libsvtav1 が avcodec_open2 で落ちる）。
        # 黙って丸めると「実際の実行レート」を偽ることになるので、明示を求める。
        raise SystemExit(
            f"採用行の間隔から算出した fps が {fps}（中央値間隔 {median_dt:.6f}s）で、"
            f"現実的な範囲 {FPS_MIN}〜{FPS_MAX} を外れている。\n"
            "  記録が短すぎるか t_sec が壊れている可能性がある。意図的なら --fps で明示する。"
        )

    meta = {
        "repo_id": args.repo_id,
        "task": args.task,
        "model_repo": args.model_repo,
        "notes": args.notes,
        "granularity": episodes[0]["granularity"],
        "granularity_reason": granularity_reason,
        "fps": fps,
        "median_dt": median_dt,
        "with_video": with_video,
        "episodes": episodes,
    }
    logging.info(f"出力: {args.repo_id} / fps={fps} / エピソード {len(episodes)} / 総フレーム {len(all_sel)}")

    if args.dry_run:
        logging.info("--dry-run のため書き込まない")
        print(json.dumps({k: v for k, v in meta.items() if k != "episodes"}, ensure_ascii=False, indent=2))
        return 0

    from lerobot.datasets.lerobot_dataset import LeRobotDataset

    root = args.root
    if root is not None:
        root = root.expanduser()
        if root.exists():
            if not args.overwrite:
                raise SystemExit(f"{root} が既にある（--overwrite で消してから作る）")
            shutil.rmtree(root)

    ds = LeRobotDataset.create(
        repo_id=args.repo_id,
        fps=fps,
        features=build_features(with_video),
        root=root,
        robot_type=ROBOT_TYPE,
        use_videos=with_video,
    )

    for e in episodes:
        for row in e["rows"]:
            frame = {
                "observation.state": np.array(
                    [float(row[f"present.{k}"]) for k in JOINT_NAMES], dtype=np.float32
                ),
                "action": np.array([float(row[f"cmd.{k}"]) for k in JOINT_NAMES], dtype=np.float32),
                "t_sec": np.array([float(row["t_sec"])], dtype=np.float32),
                "task": args.task,
            }
            if with_video:
                frame[CAMERA_KEY] = load_image(e["frame_index"][int(row["step"])])
            ds.add_frame(frame)
        ds.save_episode()
        logging.info(f"[ep{e['index']}] 保存した（{e['n_used']} フレーム）")

    write_readme(ds.root, meta)
    logging.info(f"完了: {ds.root}")

    if args.push_to_hub:
        logging.info("HF へ push する（private）")
        ds.push_to_hub(private=True)
        logging.info(f"push 完了: https://huggingface.co/datasets/{args.repo_id}")
    else:
        logging.info("push はしていない（--push-to-hub を明示すると push する）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
