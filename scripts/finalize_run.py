#!/usr/bin/env python3
"""収集セッション(run_dir)を回収前に仕上げる: ledger.csv / notes.md / setup/。

設計正本: orboh-company-vault
  07Member's-free-space/Kento/02-Tech-Plan/02VLA/collection-recording-manual.md
  (§台帳の列 / §記録ファイルの役割 / 実装バックログ4)

台帳の自動列は**収録時に計測しない**。収録側にコードを足さず、回収時に既に
ある2つのファイルから派生させる:

  start-pose-<side>.jsonl  ... エピソード毎の開始姿勢逸脱と outcome
  <dataset>/meta/episodes/ ... episode_index / tasks / length (parquet)

この2つを episode_index で left join し、人が判断する3列(success /
object_pos / note)は空で出す。judge は manifest から自動スタンプする。

データセットは Thor 上にあるので、このスクリプトも Thor で走らせる
(pull_run.sh が ssh 経由で呼ぶ)。データセットに届かない場所で走らせた場合は
jsonl 由来の列だけで台帳を出し、欠けた列を警告する。
"""

from __future__ import annotations

import argparse
import csv
import json
import re
import sys
from pathlib import Path

LEDGER_NAME = "ledger.csv"
NOTES_NAME = "notes.md"
# 自動7列 + 人手3列 + judge(自動スタンプ)。順序は設計正本 §台帳の列 のまま。
COLUMNS = [
    "ep",
    "ts",
    "task",
    "dur_s",
    "retakes",
    "worst_dev_deg",
    "gate_outcome",
    "success",
    "object_pos",
    "note",
    "judge",
]
# manual では ep0 がゲートを通らないので jsonl に行が出ない。空欄にすると
# 「計測に失敗した」と区別できないので、欠測でないことを明示する。
NA_MANUAL_EP0 = "n/a (manual ep0)"


def warn(msg: str) -> None:
    print(f"[finalize] {msg}", file=sys.stderr)


# --- 入力の読み取り ----------------------------------------------------------


def load_manifest(run_dir: Path) -> dict:
    path = run_dir / "manifest.json"
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        warn(f"manifest.json を読めない({exc})。分かる範囲で続ける: {path}")
        return {}


def read_episode_advance(run_dir: Path) -> str | None:
    """command.sh から EPISODE_ADVANCE を読む。

    manifest には入れていない。これは lerobot の設定ダンプに出る値で、
    manifest にダンプ由来の値を複製しない方針だから(§記録ファイルの役割)。
    ただし台帳側は「ep0 に行が無いのは manual だから正常」を判定したいので、
    run_dir 内で唯一この値を解決済みで持っている command.sh から読む。
    """
    path = run_dir / "command.sh"
    try:
        text = path.read_text(encoding="utf-8")
    except OSError:
        return None
    m = re.search(r"EPISODE_ADVANCE=([A-Za-z]+)", text)
    return m.group(1) if m else None


def load_start_pose_rows(run_dir: Path) -> dict[int, list[dict]]:
    """start-pose-*.jsonl を episode_index でまとめる(ファイル内の順序を保つ)。"""
    by_ep: dict[int, list[dict]] = {}
    paths = sorted(run_dir.glob("start-pose-*.jsonl"))
    if not paths:
        warn("start-pose-*.jsonl が無い。開始姿勢由来の列は空になる。")
        return by_ep
    for path in paths:
        try:
            lines = path.read_text(encoding="utf-8").splitlines()
        except OSError as exc:
            warn(f"{path.name} を読めない({exc})")
            continue
        for lineno, line in enumerate(lines, 1):
            line = line.strip()
            if not line:
                continue
            try:
                entry = json.loads(line)
            except json.JSONDecodeError:
                warn(f"{path.name}:{lineno} が JSON として読めない。飛ばす。")
                continue
            # 先頭行は正準開始姿勢の宣言(逸脱がどこから測られたか)で、
            # エピソードの行ではない。台帳には出さず読み飛ばす。
            if entry.get("type") == "start_pose_reference":
                continue
            ep = entry.get("episode_index")
            if not isinstance(ep, int):
                continue
            by_ep.setdefault(ep, []).append(entry)
    return by_ep


def load_episodes_meta(dataset_root: Path) -> tuple[dict[int, dict], float | None]:
    """meta/episodes/**.parquet と meta/info.json から ep 毎の tasks / length / fps。"""
    fps = None
    info = dataset_root / "meta" / "info.json"
    try:
        fps = json.loads(info.read_text(encoding="utf-8")).get("fps")
    except (OSError, json.JSONDecodeError):
        warn(f"info.json を読めないので fps 不明: {info}")

    episodes_dir = dataset_root / "meta" / "episodes"
    parquets = sorted(episodes_dir.rglob("*.parquet"))
    if not parquets:
        warn(f"episodes の parquet が無い: {episodes_dir}")
        return {}, fps
    try:
        import pandas as pd
    except ImportError:
        warn("pandas が無いので task / dur_s は空になる(lerobot の venv で走らせること)")
        return {}, fps

    out: dict[int, dict] = {}
    for path in parquets:
        try:
            df = pd.read_parquet(path)
        except Exception as exc:  # 壊れている等
            warn(f"{path} を読めない({exc})")
            continue
        if not {"episode_index", "length"}.issubset(df.columns):
            warn(f"{path} に episode_index/length が無い")
            continue
        # 動画区間の列（カメラごとに from/to のペア）。データと動画のズレ検出に使う。
        video_keys = [
            c[len("videos/") : -len("/from_timestamp")]
            for c in df.columns
            if c.startswith("videos/") and c.endswith("/from_timestamp")
        ]
        for _, row in df.iterrows():
            tasks = row.get("tasks")
            if hasattr(tasks, "tolist"):
                tasks = tasks.tolist()
            if isinstance(tasks, (list, tuple)):
                task = ", ".join(str(t) for t in tasks)
            else:
                task = "" if tasks is None else str(tasks)
            entry = {"task": task, "length": int(row["length"])}
            # 各カメラの動画区間をフレーム数に換算しておく。
            # 動画区間の長さは「そのエピソードで最も長かったテイク（破棄分を含む）」に
            # なりうる（破棄の残骸フレームがエンコードされる不具合・lerobot 側で根治済み）。
            # データの length と食い違っていれば、そのエピソードの映像後半は
            # **破棄したテイクの映像**であり、学習に入れてはいけない。
            if fps:
                vf = {}
                for key in video_keys:
                    try:
                        dur = float(row[f"videos/{key}/to_timestamp"]) - float(
                            row[f"videos/{key}/from_timestamp"]
                        )
                        vf[key] = round(dur * float(fps))
                    except (KeyError, TypeError, ValueError):
                        continue
                if vf:
                    entry["video_frames"] = vf
            out[int(row["episode_index"])] = entry
    return out, fps


# --- 派生 -------------------------------------------------------------------


def pick_authoritative(rows: list[dict]) -> dict | None:
    """実際に録り始めた回の行を選ぶ。

    同じ episode_index に複数行が出るのは、破棄して録り直した場合。
    `rerecord` は「破棄した」ことの記録なので、最後の非 rerecord 行が
    「その ep が実際に始まったときの姿勢」を表す。
    """
    if not rows:
        return None
    for row in reversed(rows):
        if row.get("outcome") != "rerecord":
            return row
    return rows[-1]


def count_retakes(rows: list[dict]) -> int:
    """再録回数。

    manual では破棄が `rerecord` として1行残るので、その数を数えれば足りる。
    auto にはその outcome が無く、録り直すとループが回って同じ ep の行が
    増えるだけなので、行数-1 を採る。どちらのモードでも計装は不要。
    """
    rerecords = sum(1 for row in rows if row.get("outcome") == "rerecord")
    if rerecords:
        return rerecords
    return max(0, len(rows) - 1)


def build_ledger(
    episodes: dict[int, dict],
    by_ep: dict[int, list[dict]],
    fps: float | None,
    judge: str,
    advance: str | None,
) -> list[dict]:
    # 何本録れたかはデータセットが正。届かない場合だけ jsonl の ep 番号で代用する
    # (jsonl は「開始した」記録なので、保存されなかった ep も混じりうる)。
    indices = sorted(episodes) if episodes else sorted(by_ep)
    rows = []
    for ep in indices:
        meta = episodes.get(ep, {})
        entries = by_ep.get(ep, [])
        auth = pick_authoritative(entries)

        if auth is not None:
            ts = auth.get("timestamp", "")
            worst = auth.get("worst_deviation_deg", "")
            outcome = auth.get("outcome", "")
        elif ep == 0 and advance == "manual":
            ts = worst = outcome = NA_MANUAL_EP0
        else:
            ts = worst = outcome = ""

        length = meta.get("length")
        if length is not None and fps:
            dur = f"{length / float(fps):.1f}"
        else:
            dur = ""

        # データと動画の長さの突合。不一致＝映像の後半が破棄テイクの残骸。
        note = ""
        vf = meta.get("video_frames") or {}
        bad_cams = {
            key.rsplit(".", 1)[-1]: frames
            for key, frames in vf.items()
            if length is not None and frames != length
        }
        if bad_cams:
            detail = ", ".join(f"{cam}={frames}f" for cam, frames in sorted(bad_cams.items()))
            note = f"⚠動画不整合: data={length}f / video={detail}。学習に入れる前に除去する"

        rows.append(
            {
                "ep": ep,
                "ts": ts,
                "task": meta.get("task", ""),
                "dur_s": dur,
                "retakes": count_retakes(entries) if entries else 0,
                "worst_dev_deg": worst,
                "gate_outcome": outcome,
                # 人が埋める3列。空欄のまま残っていれば台帳は未完成
                # （note は動画不整合の機械検出だけが先に書き込むことがある）。
                "success": "",
                "object_pos": "",
                "note": note,
                "judge": judge,
            }
        )
    return rows


# --- 出力 -------------------------------------------------------------------


def write_ledger(run_dir: Path, rows: list[dict]) -> Path:
    path = run_dir / LEDGER_NAME
    with path.open("w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        writer.writeheader()
        writer.writerows(rows)
    return path


def write_notes(run_dir: Path, manifest: dict, rows: list[dict], advance: str | None) -> Path:
    """所見ノートの雛形。

    frontmatter のキーは全タスクで揃える。そうしておけば Obsidian の dataview
    がフォルダを跨いで1つの表に集計でき、「操作者 A のセッションだけ」
    「シーンを変えた前後」で引ける(§なぜこれで横断参照になるか)。
    """
    identity = manifest.get("identity", {})
    people = manifest.get("people", {})
    scene = manifest.get("scene", {})
    control = manifest.get("control_variables", {})
    artifacts = manifest.get("artifacts", {})
    operator = people.get("operator") or "unknown"

    def yaml_val(value) -> str:
        if value is None:
            return "null"
        if isinstance(value, bool):
            return "true" if value else "false"
        return f'"{value}"' if isinstance(value, str) else str(value)

    lines = [
        "---",
        f"created_by: Claude ({operator})",
        f"session: {yaml_val(identity.get('session_id'))}",
        f"started_utc: {yaml_val(identity.get('started_utc'))}",
        # ローカル時刻と時刻だけの整数を dataview に出す。昼/夜はシーンプロファイルで
        # 統制していない流動的な条件なので、**セッション層で突合できるようにする**のが
        # ここの役目（`local_hour` で「昼のセッションだけ」を機械的に絞れる）。
        f"started_local: {yaml_val(identity.get('started_local'))}",
        f"local_hour: {yaml_val(identity.get('local_hour'))}",
        f"operator: {yaml_val(operator)}",
        f"judge: {yaml_val(people.get('judge'))}",
        f"scene: {yaml_val(scene.get('name'))}",
        f"scene_sha: {yaml_val((scene.get('sha256') or '')[:12] or None)}",
        f"side: {yaml_val(control.get('side'))}",
        f"fps: {yaml_val(control.get('fps'))}",
        f"gravity_scale: {yaml_val(control.get('gravity_scale'))}",
        f"repo_id: {yaml_val(artifacts.get('repo_id_stamped') or artifacts.get('repo_id_requested'))}",
        f"episodes: {yaml_val(artifacts.get('episodes_recorded') if artifacts.get('episodes_recorded') is not None else len(rows))}",
        f"episode_advance: {yaml_val(advance)}",
        # ★セッションごとに変わる条件（シーンプロファイルでは統制していないもの）。
        # 静的な物理条件はシーン側が持つが、**セッションごとに動く状態はここが正**。
        # 空のまま残っていれば「記録し忘れ」がひと目で分かる。
        "lighting_observed: null   # 昼 / 夜 / その他（started_local から自分で判断して書く）",
        "object_condition: null    # このセッション開始時のボトルの状態（凹みの進み具合）",
        # 人が台帳を埋め終えたら true にする。dataview で未完成セッションを拾う鍵。
        "ledger_complete: false",
        # 統合したら統合先データセットを書く(マージ順は manifest 側)。
        "merged_into: null",
        "tags:",
        "  - openarm",
        "  - teleop",
        "  - data-collection",
        "  - collection-session",
        "---",
        "",
        f"# 収集セッション {identity.get('session_id') or run_dir.name}",
        "",
        "> **この文書について:** このセッションで何が起きたかを人が書き残すノート。"
        "数値に出ないこと(疲れて速度が落ちた・照明が変わった・CAN を再起動した 等)を残す。"
        "条件は `manifest.json`、エピソード毎は `ledger.csv`、端末出力は `console.log`。",
        "",
        "---",
        "",
        "## 所見",
        "",
        "- ",
        "",
        "---",
        "",
        "## やり残し",
        "",
        "- [ ] `ledger.csv` の人手3列(`success` / `object_pos` / `note`)を**その日のうちに**埋める",
        "  （対象物の位置と成否は動画を見返さないと復元できず、再録回数は復元すらできない）",
        "- [ ] `manifest.json` の `human_fields`（散らした範囲・異常）を埋める",
        "- [ ] `setup/` に部屋の広角写真を1枚置く",
        "- [ ] 埋め終えたら frontmatter の `ledger_complete` を `true` にする",
        "",
    ]
    path = run_dir / NOTES_NAME
    path.write_text("\n".join(lines), encoding="utf-8")
    return path


def write_setup(run_dir: Path, manifest: dict) -> Path:
    """setup/ とその説明。人が撮るのは部屋の広角1枚だけ。"""
    control = manifest.get("control_variables", {})
    cameras = control.get("cameras") or {}
    serial_lines = [f"- `{key}` = serial `{value}`" for key, value in cameras.items()]
    setup = run_dir / "setup"
    setup.mkdir(exist_ok=True)
    body = [
        "# setup/",
        "",
        "机の配置を後から再現するための基準。**人が撮るのは部屋の広角1枚だけ**。",
        "",
        "- `room.jpg` — 机とロボットとカメラが一緒に写った引きの写真。スマホでよい",
        "",
        "残りは撮らなくてよい。カメラ視野は `lerobot-find-cameras`（オフラインで、",
        "ファイル名に serial が入る）で出せるし、開始シーンは各エピソードの mp4 の",
        "先頭フレームがそのまま証拠になる。",
        "",
        "このセッションのカメラ:",
        "",
        *(serial_lines or ["- (manifest から読めなかった)"]),
        "",
        "重いデータ本体は HF にあり、ここには置かない。",
        "",
    ]
    (setup / "README.md").write_text("\n".join(body), encoding="utf-8")
    return setup


# --- main -------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run-dir", required=True)
    parser.add_argument(
        "--dataset-root",
        default="",
        help="データセットの場所。既定は manifest.json の artifacts.dataset_root",
    )
    parser.add_argument("--force", action="store_true", help="既存の ledger.csv / notes.md を上書きする")
    args = parser.parse_args(argv)

    run_dir = Path(args.run_dir).expanduser()
    if not run_dir.is_dir():
        warn(f"run_dir が無い: {run_dir}")
        return 1

    manifest = load_manifest(run_dir)
    advance = read_episode_advance(run_dir)
    by_ep = load_start_pose_rows(run_dir)

    root_str = args.dataset_root or (manifest.get("artifacts", {}).get("dataset_root") or "")
    episodes: dict[int, dict] = {}
    fps = manifest.get("control_variables", {}).get("fps")
    if root_str:
        dataset_root = Path(root_str).expanduser()
        if dataset_root.is_dir():
            episodes, meta_fps = load_episodes_meta(dataset_root)
            fps = meta_fps or fps
        else:
            warn(f"データセットが見つからない({dataset_root})。task / dur_s は空になる。")
    else:
        warn("manifest に dataset_root が無い。--dataset-root で渡すか、先に manifest の resolve を通すこと。")

    judge = manifest.get("people", {}).get("judge") or ""
    rows = build_ledger(episodes, by_ep, fps, judge, advance)
    if not rows:
        warn("エピソードが1本も見つからない。空の台帳を書く。")

    ledger = run_dir / LEDGER_NAME
    notes = run_dir / NOTES_NAME
    if ledger.exists() and not args.force:
        warn(f"既存の {LEDGER_NAME} を残した(上書きは --force)。人手列を消さないため。")
    else:
        write_ledger(run_dir, rows)
        print(str(ledger))
    if notes.exists() and not args.force:
        warn(f"既存の {NOTES_NAME} を残した(上書きは --force)。人が書いた所見を消さないため。")
    else:
        write_notes(run_dir, manifest, rows, advance)
        print(str(notes))
    print(str(write_setup(run_dir, manifest)))

    n_mismatch = sum(1 for r in rows if "動画不整合" in (r["note"] or ""))
    if n_mismatch:
        warn(
            f"⚠ 動画とデータの長さが食い違うエピソードが {n_mismatch} 本ある"
            "（破棄テイクの残骸が映像に混入。統合や学習に入れる前に除去する）"
        )
    n_missing = sum(1 for r in rows if r["gate_outcome"] == "")
    if n_missing:
        warn(f"開始姿勢の記録が無いエピソードが {n_missing} 本ある(ゲート無効で録った可能性)")
    unhomed = [r["ep"] for r in rows if r["gate_outcome"] in ("started_unhomed", "override", "timeout")]
    if unhomed:
        warn(f"開始姿勢を揃えずに始めたエピソード: {unhomed}(学習前に本数を確認する)")
    print(f"[finalize] {len(rows)} エピソード分の台帳を用意した。人手3列は空欄。", file=sys.stderr)
    return 0


if __name__ == "__main__":
    sys.exit(main())
