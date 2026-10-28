#!/usr/bin/env python3
"""1収集セッションの環境スナップショット(manifest.json)を書く。

設計正本: orboh-company-vault
  07Member's-free-space/Kento/02-Tech-Plan/02VLA/collection-recording-manual.md

manifest は設定の全文を持たない。lerobot は起動時に解決済み設定を丸ごと
ダンプしており(stderr)、それは console.log に入っている。ここに書くのは
**ダンプから取れない新規事実6系統**だけ:

  1. openarm-orboh / lerobot の branch・commit・未コミット数
  2. python・pinocchio・lerobot の版
  3. stamp 後の実 repo_id (ダンプは stamp より前に出るので構造的に出ない)
  4. 較正ファイルの sha256 と mtime
  5. HF 身元 (push 時に追記。収録時はオフラインで取れない)
  6. マージ順と run 毎 ep 数 (統合後に台帳と突合する鍵)
  (2026-10-28 追加) タスクの台帳のタスク ID・課題コード・外に見せる ID・条件の版 (task_ledger)

加えて不一致検査のためだけに4統制変数を意図的に複製する
(side / カメラ serial / fps / GRAVITY_SCALE)。これらが複数セッションで
食い違ったらデータは混ぜられないので、ダンプを読まずに機械照合できる
場所に置いておく価値がある。それ以外は console.log へのポインタで済ます。

使い方:
  collect_manifest.py init        --run-dir DIR ... > manifest.json を書く
  collect_manifest.py resolve     --run-dir DIR     ... 実行後に artifacts を埋める
  collect_manifest.py check-scene --scene-path P    ... TODO の数だけを出す
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import platform
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

SCHEMA = "orboh-collection-manifest/1"
MANIFEST_NAME = "manifest.json"


# --- 小道具 -----------------------------------------------------------------


def _utc(ts: float) -> str:
    return datetime.fromtimestamp(ts, timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ")


def _local_time(started_utc: str) -> dict:
    """UTC の開始時刻からローカル時刻を導出する。

    shell が渡した UTC から**導出**するので、2つの値が食い違うことがない
    （それぞれ別に time を呼ぶと数秒ずれ、日付境界では日がずれる）。
    """
    try:
        dt = datetime.strptime(started_utc, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc)
    except (TypeError, ValueError):
        return {"started_local": None, "tz": None, "local_hour": None}
    local = dt.astimezone()
    return {
        "started_local": local.strftime("%Y-%m-%dT%H:%M:%S%z"),
        "tz": local.tzname(),
        # 昼夜の突合を機械でやるとき、文字列を再パースせずに済むように整数でも置く。
        "local_hour": local.hour,
    }


def _sha256(path: Path) -> str | None:
    try:
        return hashlib.sha256(path.read_bytes()).hexdigest()
    except OSError:
        return None


def _git(repo: Path, *args: str) -> str | None:
    """git を1回叩く。repo が git でない/git が無い場合は None。"""
    try:
        out = subprocess.run(
            ["git", "-C", str(repo), *args],
            capture_output=True,
            text=True,
            timeout=10,
        )
    except (OSError, subprocess.SubprocessError):
        return None
    if out.returncode != 0:
        return None
    return out.stdout.strip()


def _repo_info(repo: Path | None) -> dict:
    """branch / commit / 未コミット数。値が取れない箇所は None で埋める。

    未コミット数を持つのは、コミットされていない変更で録ったセッションは
    commit だけでは再現できない = 後から「このコードで録った」と言えないため。
    0 でないこと自体が警告になる。
    """
    if repo is None or not repo.exists():
        return {"root": str(repo) if repo else None, "branch": None, "commit": None, "dirty_files": None}
    porcelain = _git(repo, "status", "--porcelain")
    dirty = len([line for line in porcelain.splitlines() if line.strip()]) if porcelain is not None else None
    # 上流との差。fetch はしない（preflight が直前に best-effort で fetch している）。
    # 「上流より古いコードで録った」ことが後から分かるように behind を残す。
    counts = _git(repo, "rev-list", "--left-right", "--count", "@{u}...HEAD")
    behind = ahead = None
    if counts:
        try:
            behind, ahead = (int(x) for x in counts.split())
        except ValueError:
            pass
    return {
        "root": str(repo),
        "branch": _git(repo, "rev-parse", "--abbrev-ref", "HEAD"),
        "commit": _git(repo, "rev-parse", "HEAD"),
        "dirty_files": dirty,
        "behind_upstream": behind,
        "ahead_upstream": ahead,
    }


def _lerobot_paths() -> tuple[Path | None, Path | None, Path | None, str | None]:
    """lerobot の repo root / HF_LEROBOT_HOME / 較正ルート / 版。

    定数は lerobot 自身から読む。環境変数 HF_LEROBOT_HOME /
    HF_LEROBOT_CALIBRATION での上書きを自前で再実装すると必ずずれるため。
    """
    try:
        import lerobot
        from lerobot.utils.constants import HF_LEROBOT_CALIBRATION, HF_LEROBOT_HOME
    except Exception:
        return None, None, None, None
    mod = Path(lerobot.__file__).resolve()
    # src/lerobot/__init__.py -> repo root は2つ上
    root = mod.parents[2] if len(mod.parents) >= 3 else None
    version = getattr(lerobot, "__version__", None)
    return root, Path(HF_LEROBOT_HOME), Path(HF_LEROBOT_CALIBRATION), version


def _calibration_entries(calib_root: Path | None, side: str) -> list[dict]:
    """follower/leader の較正ファイルの sha256 と mtime。

    較正が変わると開始姿勢の基準そのものが変わるので、どの較正で録ったかは
    データの意味を決める。ファイルが無い場合も「無かった」ことを記録する
    (存在しない = 恒久零が起動していない可能性の手がかりになる)。
    """
    # side="bimanual" は左右 4 ユニットぶん (bi_openarm_* は内部で follower_left/right・
    # leader_left/right の較正を読むので、記録すべきファイルも単腕と同じ場所にある)。
    sides = ["left", "right"] if side == "bimanual" else [side]
    specs = []
    for s_ in sides:
        specs.append(("robot", "robots", "openarm_follower", f"follower_{s_}"))
        specs.append(("teleop", "teleoperators", "openarm_leader", f"leader_{s_}"))
    entries = []
    for role, bucket, type_name, dev_id in specs:
        entry = {"role": role, "type": type_name, "id": dev_id, "path": None, "sha256": None, "mtime_utc": None}
        if calib_root is not None:
            path = calib_root / bucket / type_name / f"{dev_id}.json"
            entry["path"] = str(path)
            if path.is_file():
                entry["sha256"] = _sha256(path)
                entry["mtime_utc"] = _utc(path.stat().st_mtime)
        entries.append(entry)
    return entries


def _initial_pose_block(pose_path: Path | None) -> dict:
    """teleop 起動時に整列先として使う開始姿勢 YAML の sha256 と mtime。

    このファイルは git 未管理のままだと撮り直しを検出できない——シーン
    プロファイルと同じ穴（撮り直すと50epが2条件に割れても、各 run 自体は
    正しく記録されたまま学習の分析時まで発覚しない）。sha を manifest に
    残すことで check-pose-drift が同じ仕組みで検出できるようにする。
    """
    block = {"path": None, "sha256": None, "mtime_utc": None}
    if pose_path is None or not pose_path.is_file():
        return block
    block["path"] = str(pose_path)
    block["sha256"] = _sha256(pose_path)
    block["mtime_utc"] = _utc(pose_path.stat().st_mtime)
    return block


def _scene_block(scene_name: str | None, scene_path: Path | None) -> dict:
    """シーンプロファイルの名前と sha、そして未確定(TODO)の数。

    sha を持つ理由: 名前は同じまま中身を書き換えられてしまうため、名前だけでは
    「同じ物理条件で録った」を保証できない。sha が一致すれば同一と機械で言える。

    todo_count を持つ理由: TODO のまま録ると、そのセッションは「まだ値の入って
    いない版」の sha を指す。あとで値を埋めると sha が変わり、そのセッションだけ
    存在しない条件を指したまま孤立する。数を残しておけば後から検出できる。
    """
    block = {"name": scene_name, "path": None, "sha256": None, "todo_count": None}
    if scene_name in (None, "", "none"):
        block["name"] = None
        return block
    if scene_path is None or not scene_path.is_file():
        return block
    block["path"] = str(scene_path)
    block["sha256"] = _sha256(scene_path)
    block["todo_count"] = count_scene_todos(scene_path)
    return block


def count_scene_todos(scene_path: Path) -> int | None:
    """値が TODO のままのキーを数える。yaml が無ければ素朴に行を数える。"""
    try:
        import yaml
    except Exception:
        try:
            text = scene_path.read_text(encoding="utf-8")
        except OSError:
            return None
        return sum(1 for line in text.splitlines() if not line.lstrip().startswith("#") and ": TODO" in line)
    try:
        data = yaml.safe_load(scene_path.read_text(encoding="utf-8"))
    except Exception:
        return None

    def walk(node):
        if isinstance(node, dict):
            for value in node.values():
                yield from walk(value)
        elif isinstance(node, list):
            for value in node:
                yield from walk(value)
        elif node == "TODO":
            yield 1

    return sum(walk(data))


def _task_ledger_block(args: argparse.Namespace) -> dict | None:
    """タスク ID・課題コード・外に見せる ID・条件の版（どれも record が scripts/task.py name から受けて渡す）。"""
    if not args.task_id:
        return None
    return {
        "task": args.task_id,
        "issue": args.issue or None,
        "external_id": args.external_id or None,
        "condition_version": args.condition_version or None,
        "ledger": f"config/tasks/{args.task_id}.yaml",
    }


# --- init -------------------------------------------------------------------


def cmd_init(args: argparse.Namespace) -> int:
    run_dir = Path(args.run_dir)
    run_dir.mkdir(parents=True, exist_ok=True)

    lerobot_root, _hf_home, calib_root, lerobot_version = _lerobot_paths()
    if args.lerobot_root:
        lerobot_root = Path(args.lerobot_root)

    try:
        import pinocchio

        pin_version = getattr(pinocchio, "__version__", "unknown")
    except Exception:
        # Vector には入っていない(重力補償は Thor だけ)。取れないことを記録する。
        pin_version = None

    scene_path = Path(args.scene_path) if args.scene_path else None
    scene = _scene_block(args.scene, scene_path)

    pose_path = Path(args.initial_pose_path) if args.initial_pose_path else None
    initial_pose = _initial_pose_block(pose_path)
    # bimanual: 右 YAML を従来キー initial_pose に、左 YAML を initial_pose_left に置く
    # (check-pose-drift は両キーを見る)。
    pose_left_path = Path(args.initial_pose_path_left) if args.initial_pose_path_left else None
    initial_pose_left = _initial_pose_block(pose_left_path) if pose_left_path else None

    cameras = {"chest": args.cam_serial}
    if args.wrist_serial:
        cameras[f"wrist_{args.side}"] = args.wrist_serial
    if args.wrist_right_serial:
        cameras["wrist_right"] = args.wrist_right_serial
    if args.wrist_left_serial:
        cameras["wrist_left"] = args.wrist_left_serial

    manifest = {
        "schema": SCHEMA,
        "identity": {
            "session_id": args.session_id,
            "run_dir": str(run_dir),
            "started_utc": args.started_utc,
            # ローカル時刻も併記する。UTC は「別マシンで読み・別日と順序を比較する」ために
            # 必要だが、**昼か夜かは UTC を見ても分からない**(JST なら +9 が要る)。
            # 昼/夜はシーンプロファイルでは統制していない流動的な条件なので、後から
            # 「あのセッションは昼だったか」を答えられるのはこの値だけになる。
            # 分類(昼/夜)は焼き込まない——閾値は恣意的で季節でも動くので、**分類ではなく
            # 分類できる素の値**を残す。
            **_local_time(args.started_utc),
            "host": platform.node(),
        },
        "people": {
            # operator を必須にするのは、操作者が違えばデモの癖が変わり、
            # 少数データの模倣学習では交絡要因になるため。
            "operator": args.operator,
            "judge": args.judge or None,
        },
        # タスクの台帳（config/tasks/<task>.yaml・2026-10-28）で録ったセッションだけ持つ。台帳の無いタスクは null。
        # データセットの名前（repo_id）にも外に見せる ID と条件の版は入っているが、名前を文字列で割らずに
        # 「どの課題コードのセッションか」を機械で引けるように、別々の項目として置く。操作者は people.operator。
        "task_ledger": _task_ledger_block(args),
        "scene": scene,
        "initial_pose": initial_pose,
        **({"initial_pose_left": initial_pose_left} if initial_pose_left is not None else {}),
        "code": {
            "openarm_orboh": _repo_info(Path(args.repo_root) if args.repo_root else None),
            "lerobot": _repo_info(lerobot_root),
        },
        "env": {
            "python": platform.python_version(),
            "pinocchio": pin_version,
            "lerobot": lerobot_version,
            "platform": platform.platform(),
        },
        "calibration": _calibration_entries(calib_root, args.side),
        # 不一致検査用の意図的な複製。ここが食い違うセッションは混ぜられない。
        "control_variables": {
            "side": args.side,
            "cameras": cameras,
            "fps": args.fps,
            "gravity_scale": args.gravity_scale,
        },
        "artifacts": {
            "repo_id_requested": args.repo_id,
            # stamp 後の実 repo_id。ダンプは stamp より前に出るので実行後に解決する。
            "repo_id_stamped": None,
            "dataset_root": None,
            "episodes_recorded": None,
            "revision": None,
            # HF 身元とマージ情報は収録時に取れない(オフライン / 統合はあとの工程)。
            "hf_identity": None,
            "merge_order": None,
            "exit_code": None,
        },
        # 人があとで埋める欄。空のまま残っていること自体が回収漏れの印になる。
        # 後半2つは**セッションごとに変わる条件**で、静的なシーンプロファイルには
        # 置けないもの（置くと「一度宣言して sha で参照する」モデルが壊れる）。
        "human_fields": {
            "scatter_actual": None,
            "anomalies": None,
            # 昼/夜。identity.started_local から人が判断して書く（分類は焼き込まない）。
            "lighting_observed": None,
            # このセッション開始時のボトルの状態。凹みは収集を通じて進む。
            "object_condition": None,
        },
        "pointers": {
            "console_log": "console.log",
            "start_pose_log": f"start-pose-{args.side}.jsonl",
            "command": "command.sh",
            "note": (
                "解決済み設定の全文は console.log の設定ダンプにある。"
                "manifest はそれを再記録せず、ダンプから取れない事実だけを持つ。"
            ),
        },
    }

    out = run_dir / MANIFEST_NAME
    out.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")

    # 呼び出し側(shell)が判断に使えるように、警告は stderr へ、要点は stdout へ。
    print(str(out))
    todo = scene.get("todo_count")
    if todo:
        print(
            f"[manifest] 警告: シーンプロファイルに TODO が {todo} 件残っている: {scene.get('path')}",
            file=sys.stderr,
        )
    for repo_key in ("openarm_orboh", "lerobot"):
        dirty = manifest["code"][repo_key]["dirty_files"]
        if dirty:
            print(
                f"[manifest] 警告: {repo_key} に未コミットの変更が {dirty} 件ある"
                "(commit だけではこのセッションを再現できない)",
                file=sys.stderr,
            )
    for repo_key in ("openarm_orboh", "lerobot"):
        behind = manifest["code"][repo_key].get("behind_upstream")
        if behind:
            print(
                f"[manifest] 警告: {repo_key} は上流より {behind} コミット古い"
                "(push 済みの修正が取り込まれていない可能性)",
                file=sys.stderr,
            )
    for entry in manifest["calibration"]:
        if entry["sha256"] is None:
            print(
                f"[manifest] 警告: 較正ファイルが無い: {entry['path']}"
                "(恒久零が起動していない可能性)",
                file=sys.stderr,
            )
    if args.initial_pose_path and initial_pose["sha256"] is None:
        print(
            f"[manifest] 警告: 開始姿勢ファイルが無い: {args.initial_pose_path}"
            "(公式既定の開始姿勢にフォールバックしている)",
            file=sys.stderr,
        )
    return 0


# --- resolve ----------------------------------------------------------------


def _find_stamped(hf_home: Path, repo_id: str, started_ts: float) -> list[Path]:
    """stamp 後のデータセットディレクトリを探す。

    stamp_repo_id() は repo_id に `_YYYYmmdd_HHMMSS` を **ローカル時刻で**
    足す(configs/dataset.py)。run_dir は UTC 名なので文字列では突き合わせ
    られない。よって「要求した repo_id で始まり、この run の開始以降に
    作られたディレクトリ」で引く。
    """
    if "/" in repo_id:
        org, name = repo_id.split("/", 1)
        parent = hf_home / org
    else:
        parent = hf_home
        name = repo_id
    if not parent.is_dir():
        return []
    hits = []
    for child in parent.iterdir():
        if not child.is_dir() or not child.name.startswith(f"{name}_"):
            continue
        try:
            if child.stat().st_mtime + 1 < started_ts:
                continue
        except OSError:
            continue
        hits.append(child)
    return sorted(hits, key=lambda p: p.stat().st_mtime)


def _episode_count(dataset_root: Path) -> int | None:
    info = dataset_root / "meta" / "info.json"
    try:
        return json.loads(info.read_text(encoding="utf-8")).get("total_episodes")
    except Exception:
        return None


def cmd_check_drift(args: argparse.Namespace) -> int:
    """同じシーン名で前に録った run と sha が一致するかを見る。

    「1本録ったらシーンを書き換えず新しいファイルを作る」は規約として書いてあるが、
    **機構が無いと守られたか誰も確認できない**。50ep は複数セッション・複数日に
    分かれるので、途中で実測値を1つ直すと前後で sha が割れ、各 run の記録は正しい
    まま「50ep が実は2条件に割れていた」と学習の分析時に判明する——そこまで行くと
    もう録り直しでしか直せない。

    出力（shell がそのまま判定に使う）:
      一致・前例なし・判定不能 → 何も出さず 0
      食い違い               → "DRIFT <前の sha 先頭12> <今の sha 先頭12> <前の run 名>"
    """
    scene_path = Path(args.scene_path).expanduser()
    if not scene_path.is_file():
        return 0
    current = _sha256(scene_path)
    if not current:
        return 0
    name = scene_path.stem

    root = Path(args.collections_dir).expanduser()
    if not root.is_dir():
        return 0
    # 最新の run と比べる。全 run と比べないのは、既に割れている履歴があると
    # 毎回止まって進めなくなるため（直近と揃っていれば運用としては足りる）。
    newest = None
    for manifest_path in sorted(root.glob("*/manifest.json")):
        try:
            m = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        scene = m.get("scene") or {}
        if scene.get("name") != name or not scene.get("sha256"):
            continue
        started = (m.get("identity") or {}).get("started_utc") or ""
        if newest is None or started >= newest[0]:
            newest = (started, scene["sha256"], manifest_path.parent.name)
    if newest is None:
        return 0
    if newest[1] != current:
        print(f"DRIFT {newest[1][:12]} {current[:12]} {newest[2]}")
    return 0


def _pose_key(path: Path) -> str:
    """開始姿勢ファイルの照合キー。開始姿勢セット（config/pose_sets/<set>/<side>.yaml・2026-09-30〜）は
    "<set>/<side>"、それ以前の固定名（config/initial_pose_<side>.yaml）は stem。セットのファイル名は
    どれも left/right なので、stem だけで照合すると別セットの run と比べてしまう。"""
    if path.parent.parent.name == "pose_sets":
        return f"{path.parent.name}/{path.stem}"
    return path.stem


def cmd_check_pose_drift(args: argparse.Namespace) -> int:
    """同じ開始姿勢ファイルで前に録った run と sha が一致するかを見る。

    cmd_check_drift と同じ理由・同じ契約（DRIFT 出力の書式も揃える）。開始姿勢
    YAML は較正と違い git 未管理で撮り直しの検出手段が無かったため、シーン
    プロファイルに倣ってここで機械照合できるようにする。
    """
    pose_path = Path(args.initial_pose_path).expanduser()
    if not pose_path.is_file():
        return 0
    current = _sha256(pose_path)
    if not current:
        return 0
    name = _pose_key(pose_path)

    root = Path(args.collections_dir).expanduser()
    if not root.is_dir():
        return 0
    newest = None
    for manifest_path in sorted(root.glob("*/manifest.json")):
        try:
            m = json.loads(manifest_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        # 単腕/右は initial_pose、bimanual の左は initial_pose_left に入る。stem で照合する。
        pose = None
        for key in ("initial_pose", "initial_pose_left"):
            cand = m.get(key) or {}
            cand_path = cand.get("path")
            if cand_path and _pose_key(Path(cand_path)) == name and cand.get("sha256"):
                pose = cand
                break
        if pose is None:
            continue
        started = (m.get("identity") or {}).get("started_utc") or ""
        if newest is None or started >= newest[0]:
            newest = (started, pose["sha256"], manifest_path.parent.name)
    if newest is None:
        return 0
    if newest[1] != current:
        print(f"DRIFT {newest[1][:12]} {current[:12]} {newest[2]}")
    return 0


def cmd_check_scene(args: argparse.Namespace) -> int:
    """TODO の数だけを stdout に出す。shell 側のガードから呼ぶ。

    数えられなかった場合は何も出さない(shell は「不明」として扱い、
    ガードを掛けずに進む。数えられないことで収集を止めるのは筋が悪い)。
    """
    path = Path(args.scene_path)
    if not path.is_file():
        return 0
    count = count_scene_todos(path)
    if count is None:
        return 0
    print(count)
    return 0


def cmd_resolve(args: argparse.Namespace) -> int:
    run_dir = Path(args.run_dir)
    path = run_dir / MANIFEST_NAME
    try:
        manifest = json.loads(path.read_text(encoding="utf-8"))
    except OSError:
        print(f"[manifest] resolve 対象が無い: {path}", file=sys.stderr)
        return 0  # 収集自体は終わっているので、ここで失敗させない
    except json.JSONDecodeError as exc:
        print(f"[manifest] manifest.json が壊れている({exc}): {path}", file=sys.stderr)
        return 0

    artifacts = manifest.setdefault("artifacts", {})
    if args.exit_code is not None:
        artifacts["exit_code"] = args.exit_code

    _root, hf_home, _calib, _ver = _lerobot_paths()
    repo_id = artifacts.get("repo_id_requested")
    started = manifest.get("identity", {}).get("started_utc")
    try:
        started_ts = datetime.strptime(started, "%Y-%m-%dT%H:%M:%SZ").replace(tzinfo=timezone.utc).timestamp()
    except (TypeError, ValueError):
        started_ts = 0.0

    if hf_home is not None and repo_id:
        hits = _find_stamped(hf_home, repo_id, started_ts)
        if len(hits) == 1:
            chosen = hits[0]
        elif len(hits) > 1:
            chosen = hits[-1]
            print(
                "[manifest] 警告: この run 以降に作られた候補が複数ある。最新を採った: "
                + ", ".join(p.name for p in hits),
                file=sys.stderr,
            )
        else:
            chosen = None
            print(
                f"[manifest] 警告: stamp 後のデータセットが見つからない(要求 repo_id={repo_id})。"
                "収集が1本も保存されなかったか、HF_LEROBOT_HOME が違う。",
                file=sys.stderr,
            )
        if chosen is not None:
            artifacts["dataset_root"] = str(chosen)
            org = repo_id.split("/", 1)[0] if "/" in repo_id else None
            artifacts["repo_id_stamped"] = f"{org}/{chosen.name}" if org else chosen.name
            artifacts["episodes_recorded"] = _episode_count(chosen)

    path.write_text(json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(str(path))
    return 0


# --- CLI --------------------------------------------------------------------


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest="cmd", required=True)

    init = sub.add_parser("init", help="収集の前に manifest.json を書く")
    init.add_argument("--run-dir", required=True)
    init.add_argument("--session-id", required=True)
    init.add_argument("--started-utc", required=True)
    init.add_argument("--operator", required=True)
    init.add_argument("--judge", default="")
    init.add_argument("--scene", default="")
    init.add_argument("--scene-path", default="")
    init.add_argument("--initial-pose-path", default="")
    init.add_argument("--initial-pose-path-left", default="", help="bimanual: 左腕の開始姿勢 YAML (右は --initial-pose-path)")
    init.add_argument("--side", required=True, help="left | right | bimanual")
    init.add_argument("--cam-serial", required=True)
    init.add_argument("--wrist-serial", default="", help="単腕: wrist_<side> の D405 serial")
    init.add_argument("--wrist-right-serial", default="", help="bimanual: wrist_right の D405 serial")
    init.add_argument("--wrist-left-serial", default="", help="bimanual: wrist_left の D405 serial")
    init.add_argument("--fps", type=int, required=True)
    init.add_argument("--gravity-scale", type=float, required=True)
    init.add_argument("--repo-id", required=True)
    init.add_argument("--repo-root", default="")
    init.add_argument("--lerobot-root", default="")
    init.add_argument("--task-id", default="", help="タスクの台帳のタスク ID（例 takeei_wood）。空なら task_ledger は null")
    init.add_argument("--issue", default="", help="課題コード（台帳の書き方・例 T1-L2）")
    init.add_argument("--external-id", default="", help="外に見せる ID（例 takeei_wood_t1-l2）")
    init.add_argument("--condition-version", default="", help="条件の版（台帳の current_version・例 v2）")
    init.set_defaults(func=cmd_init)

    res = sub.add_parser("resolve", help="収集の後に stamp 後 repo_id と ep 数を埋める")
    res.add_argument("--run-dir", required=True)
    res.add_argument("--exit-code", type=int, default=None)
    res.set_defaults(func=cmd_resolve)

    chk = sub.add_parser("check-scene", help="シーンプロファイルの TODO 件数を出す")
    chk.add_argument("--scene-path", required=True)
    chk.set_defaults(func=cmd_check_scene)

    drift = sub.add_parser("check-drift", help="同じシーン名の前 run と sha が一致するか見る")
    drift.add_argument("--scene-path", required=True)
    drift.add_argument("--collections-dir", required=True)
    drift.set_defaults(func=cmd_check_drift)

    pose_drift = sub.add_parser("check-pose-drift", help="同じ開始姿勢ファイルの前 run と sha が一致するか見る")
    pose_drift.add_argument("--initial-pose-path", required=True)
    pose_drift.add_argument("--collections-dir", required=True)
    pose_drift.set_defaults(func=cmd_check_pose_drift)

    args = parser.parse_args(argv)
    return args.func(args)


if __name__ == "__main__":
    sys.exit(main())
