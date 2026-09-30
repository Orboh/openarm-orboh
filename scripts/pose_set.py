#!/usr/bin/env python3
"""開始姿勢セット（config/pose_sets/<name>/）の管理。標準ライブラリだけで動く（どの python でもよい）。

セット = 1 モデルの学習データを録った時の条件の組:
  left.yaml / right.yaml   開始姿勢（waypoints つき可）
  calib/*.json             較正ファイルの写し（follower/leader × left/right）
  scene.yaml               シーンプロファイル（scenes/<scene>.yaml）の写し
  set.yaml                 上の sha・コードの版・データセット・モデル・指示文（1 行 1 キー）

状態は 2 つ:
  draft   まだ本番の収集をしていない。収集を始めるたびに今の較正・シーン・コードの版を取り込み直す（sync）
  frozen  本番の収集を 1 回でも始めた。以後、開始姿勢・較正・シーンが 1 つでも違えば収集も推論も止まる。
          較正をやり直した・シーンを変えた・姿勢を撮り直した → このセットは残したまま `new --from` で新しいセットを作る

使い方（~/projects/openarm-orboh）:
  python3 scripts/pose_set.py list
  python3 scripts/pose_set.py show <set>
  python3 scripts/pose_set.py new <新しい名前> --from <既存セット> [--scene <scene>]     # 姿勢を引き継いで draft を作る
  python3 scripts/pose_set.py new <新しい名前> --poses-dir <dir> [--scene <scene>] [--mirror false]
  python3 scripts/pose_set.py sync <set> [--scene <scene>]           # draft に今の較正・シーン・版を取り込む
  python3 scripts/pose_set.py check <set> --mode record|rollout|teleop [--scene <scene>]
  python3 scripts/pose_set.py freeze <set> --from-run <collections/run_dir>   # record が自動で呼ぶ
  python3 scripts/pose_set.py set-model <set> --model-dir <dir> [--model-hf <repo>] [--lang "<指示文>"]
  python3 scripts/pose_set.py restore-calib <set>                    # セットの較正を戻す（今の較正は .bak-<日時> へ退避）
  python3 scripts/pose_set.py current [<set>]                        # CURRENT を表示／切り替え
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import shutil
import subprocess
import sys
from datetime import datetime
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
ROOT = Path(os.environ.get("PS_ROOT", REPO / "config/pose_sets"))
CALIB_ROOT = Path(os.environ.get("CALIB_ROOT", Path.home() / ".cache/huggingface/lerobot/calibration"))
LEROBOT = Path(os.environ.get("LEROBOT_REPO", Path.home() / "projects/lerobot"))
UNITS = {  # set.yaml のキー → (較正ファイル, calib/ 内の名前)
    "calib_follower_left": ("robots/openarm_follower/follower_left.json", "follower_left.json"),
    "calib_follower_right": ("robots/openarm_follower/follower_right.json", "follower_right.json"),
    "calib_leader_left": ("teleoperators/openarm_leader/leader_left.json", "leader_left.json"),
    "calib_leader_right": ("teleoperators/openarm_leader/leader_right.json", "leader_right.json"),
}
HEADER = "# 開始姿勢セット。scripts/pose_set.py が読み書きする（1 行 1 キー・入れ子なし）。書式と運用は config/pose_sets/README.md\n"


def sha12(p: Path) -> str:
    try:
        return hashlib.sha256(p.read_bytes()).hexdigest()[:12]
    except OSError:
        return ""


def now() -> str:
    return datetime.now().strftime("%Y-%m-%d %H:%M")


class Set:
    def __init__(self, name: str):
        self.name, self.dir = name, ROOT / name
        self.f = self.dir / "set.yaml"
        self.kv: dict[str, str] = {}
        if self.f.exists():
            for line in self.f.read_text(encoding="utf-8").splitlines():
                if line.startswith("#") or ":" not in line:
                    continue
                k, v = line.split(":", 1)
                self.kv[k.strip()] = v.strip()

    def exists(self) -> bool:
        return self.f.exists()

    def get(self, k: str) -> str:
        return self.kv.get(k, "")

    def save(self) -> None:
        self.dir.mkdir(parents=True, exist_ok=True)
        self.f.write_text(HEADER + "".join(f"{k}: {v}\n" for k, v in self.kv.items()), encoding="utf-8")

    @property
    def status(self) -> str:
        return self.get("status") or "frozen"  # status の無い旧セットは凍結扱い（学習済みモデルの組）

    def pose_files(self) -> dict[str, Path]:
        return {s: self.dir / f"{s}.yaml" for s in ("left", "right") if (self.dir / f"{s}.yaml").exists()}


def load(name: str) -> Set:
    s = Set(name)
    if not s.exists():
        sys.exit(f"[pose-set] セットが無い: {s.dir}（候補: {' '.join(names())}）")
    return s


def names() -> list[str]:
    return sorted(p.parent.name for p in ROOT.glob("*/set.yaml"))


def git_ver(repo: Path) -> str:
    try:
        h = subprocess.run(["git", "-C", str(repo), "rev-parse", "--short=8", "HEAD"], capture_output=True, text=True).stdout.strip()
        d = subprocess.run(["git", "-C", str(repo), "status", "--porcelain", "--untracked-files=no"], capture_output=True, text=True).stdout.strip()
        return (h + ("+dirty" if d else "")) if h else "n/a"
    except OSError:
        return "n/a"


# ---- sync / new ---------------------------------------------------------------
def do_sync(s: Set, scene: str | None) -> None:
    if s.status != "draft":
        sys.exit(f"[pose-set] {s.name} は凍結済み（{s.get('frozen_at') or '学習済みの組'}）。取り込み直さない。"
                 f"条件を変えたなら: python3 scripts/pose_set.py new <新しい名前> --from {s.name}")
    for side, p in s.pose_files().items():
        s.kv[f"pose_{side}_sha"] = sha12(p)
    (s.dir / "calib").mkdir(exist_ok=True)
    for key, (rel, local) in UNITS.items():
        src = CALIB_ROOT / rel
        if src.exists():
            shutil.copy2(src, s.dir / "calib" / local)
            s.kv[key] = sha12(src)
    s.kv["calib_status"] = "ok"
    scene = scene or s.get("scene")
    if scene and scene != "none":
        src = REPO / "scenes" / f"{scene}.yaml"
        if not src.exists():
            sys.exit(f"[pose-set] シーンが無い: {src}")
        shutil.copy2(src, s.dir / "scene.yaml")
        s.kv["scene"], s.kv["scene_sha"] = scene, sha12(src)
    s.kv["code_openarm_orboh"] = git_ver(REPO)
    s.kv["code_lerobot"] = git_ver(LEROBOT)
    s.kv["synced_at"] = now()
    s.save()
    print(f"[pose-set] {s.name}（draft）へ今の較正・シーン（{s.get('scene') or 'なし'}）・版を取り込んだ")


def cmd_new(a) -> int:
    s = Set(a.name)
    if s.exists():
        sys.exit(f"[pose-set] 既にある: {s.dir}")
    adopt = not a.src and not a.poses_dir and s.dir.is_dir()  # 撮影ツールが姿勢だけ書いたフォルダを取り込む
    if not adopt and bool(a.src) == bool(a.poses_dir):
        sys.exit("[pose-set] --from <セット> か --poses-dir <dir> のどちらか 1 つ（姿勢だけ置いたフォルダなら引数なしで取り込む）")
    src_dir = s.dir if adopt else ((ROOT / a.src) if a.src else Path(a.poses_dir).expanduser())
    s.dir.mkdir(parents=True, exist_ok=adopt)
    copied = []
    for side in ("left", "right"):
        p = src_dir / f"{side}.yaml"
        if p.exists():
            if not adopt:
                shutil.copy2(p, s.dir / f"{side}.yaml")
            copied.append(side)
    if not copied:
        if not adopt:
            shutil.rmtree(s.dir)
        sys.exit(f"[pose-set] {src_dir} に left.yaml / right.yaml が無い")
    base = Set(a.src) if a.src else None
    s.kv = {"name": a.name, "status": "draft", "created_at": now(),
            "derived_from": a.src or ("撮影ツールの出力" if adopt else str(src_dir)),
            "arms": "both" if len(copied) == 2 else copied[0]}
    mirror = a.mirror or (base.get("mirror") if base else "")
    if mirror:
        s.kv["mirror"] = mirror
    for k in ("freeze_side",):
        if base and base.get(k):
            s.kv[k] = base.get(k)
    s.save()
    do_sync(Set(a.name), a.scene or (base.get("scene") if base else None))
    print(f"[pose-set] 作成: {s.dir}（姿勢 {'/'.join(copied)} は {src_dir} から）。使うなら: python3 scripts/pose_set.py current {a.name}")
    return 0


def cmd_sync(a) -> int:
    do_sync(load(a.name), a.scene)
    return 0


# ---- check ----------------------------------------------------------------------
def cmd_check(a) -> int:
    s = load(a.name)
    if a.mode == "record" and s.status == "draft":
        do_sync(s, a.scene)
        return 0
    bad: list[str] = []
    warn_only = a.mode == "teleop" or os.environ.get("ALLOW_CALIB_MISMATCH") == "true"
    for side, p in s.pose_files().items():
        want = s.get(f"pose_{side}_sha")
        if want and sha12(p) != want:
            bad.append(f"開始姿勢 {side}.yaml が凍結時と違う（今 {sha12(p)} / 凍結時 {want}）")
    notes: list[str] = []
    st = s.get("calib_status")
    if st == "reconstructed":
        notes.append(f"収集時の較正は記録から復元したもの（{s.get('calib_copy')}）")
    elif st and st != "ok":
        bad.append(f"収集時の較正の状態: {st}（{s.get('calib_copy') or s.get('note')}）")
    for key, (rel, local) in UNITS.items():
        want = s.get(key)
        got = sha12(CALIB_ROOT / rel)
        # 原本の sha か、セットに入れた写し（復元版を含む）の sha のどちらかなら「収集時の較正」とみなす
        if want and got != want and got != sha12(s.dir / "calib" / local):
            bad.append(f"較正 {key[6:]} が収集時と違う（今 {sha12(CALIB_ROOT / rel) or 'ファイル無し'} / 収集時 {want}）"
                       f" → 収集時の較正に戻すなら: python3 scripts/pose_set.py restore-calib {s.name}")
    if a.mode == "record":
        if a.scene and a.scene != "none" and s.get("scene") and a.scene != s.get("scene"):
            bad.append(f"SCENE={a.scene} だがセットのシーンは {s.get('scene')}")
        sc = s.get("scene")
        if sc and s.get("scene_sha") and sha12(REPO / "scenes" / f"{sc}.yaml") != s.get("scene_sha"):
            bad.append(f"scenes/{sc}.yaml が凍結時と違う（今 {sha12(REPO / 'scenes' / f'{sc}.yaml')} / 凍結時 {s.get('scene_sha')}）")
    if a.mode == "rollout" and not s.get("model_dir"):
        bad.append("モデル未登録（学習後に set-model で登録する）")
    for n_ in notes:
        print(f"[pose-set] {s.name}: 注意: {n_}", file=sys.stderr)
    if not bad:
        print(f"[pose-set] {s.name}（{s.status}）: 開始姿勢・較正{'・シーン' if a.mode == 'record' else ''}とも一致")
        return 0
    for b in bad:
        print(f"[pose-set] {s.name}: {b}", file=sys.stderr)
    if warn_only:
        print(f"[pose-set] 警告のみで続行（mode={a.mode}）", file=sys.stderr)
        return 0
    print(f"[pose-set] 止める。{s.name} は凍結済み＝この組でしか録らない・回さない。"
          f"条件を変えたなら新しいセット: python3 scripts/pose_set.py new <名前> --from {s.name}"
          "（較正だけ承知の上で進めるなら ALLOW_CALIB_MISMATCH=true）", file=sys.stderr)
    return 1


# ---- freeze / set-model / restore-calib / current ---------------------------
def cmd_freeze(a) -> int:
    s = load(a.name)
    run = Path(a.from_run).expanduser()
    try:
        m = json.loads((run / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError):
        print(f"[pose-set] manifest が読めない: {run}（凍結しない）", file=sys.stderr)
        return 0
    art = m.get("artifacts") or {}
    ds = art.get("repo_id_stamped")
    if not ds or (m.get("scene") or {}).get("name") in (None, "", "none"):
        print("[pose-set] データセットが作られていない／SCENE=none の run なので凍結しない")
        return 0
    first = s.status == "draft"
    if first:
        s.kv["status"], s.kv["frozen_at"] = "frozen", now()
    have = [x for x in s.get("datasets").split(",") if x]
    if ds not in have:
        s.kv["datasets"] = ",".join(have + [ds])
    runs = [x for x in s.get("collections").split(",") if x]
    if run.name not in runs:
        s.kv["collections"] = ",".join(runs + [run.name])
    s.save()
    print(f"[pose-set] {s.name}: {'凍結した' if first else '凍結済み'}・データセット {ds} を記録")
    return 0


def cmd_set_model(a) -> int:
    s = load(a.name)
    s.kv["model_dir"] = a.model_dir
    if a.model_hf:
        s.kv["model_hf"] = a.model_hf
    if a.lang:
        s.kv["lang"] = a.lang
    s.save()
    print(f"[pose-set] {s.name}: model_dir={a.model_dir}")
    return 0


def cmd_restore_calib(a) -> int:
    s = load(a.name)
    stamp = datetime.now().strftime("%Y%m%d-%H%M%S")
    n = 0
    for key, (rel, local) in UNITS.items():
        src, dst = s.dir / "calib" / local, CALIB_ROOT / rel
        if not src.exists():
            continue
        bak = dst.with_name(dst.name + f".bak-{stamp}")
        k = 1
        while bak.exists():  # 同じ秒に 2 回戻しても前の退避を上書きしない
            bak, k = dst.with_name(dst.name + f".bak-{stamp}-{k}"), k + 1
        if dst.exists():
            shutil.copy2(dst, bak)
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)
        n += 1
        print(f"[pose-set] {local} を戻した（前の較正は {bak.name}）")
    if not n:
        print(f"[pose-set] {s.name} に較正の写しが無い", file=sys.stderr)
        return 1
    return 0


def cmd_current(a) -> int:
    cur = ROOT / "CURRENT"
    if a.name:
        load(a.name)
        cur.write_text(a.name + "\n", encoding="utf-8")
    print(cur.read_text(encoding="utf-8").strip())
    return 0


def cmd_list(a) -> int:
    cur = (ROOT / "CURRENT").read_text(encoding="utf-8").strip() if (ROOT / "CURRENT").exists() else ""
    for n in names():
        s = Set(n)
        print(f"{'*' if n == cur else ' '} {n:28s} {s.status:6s} scene={s.get('scene') or '-':40s} model={s.get('model_dir') or '-'}")
    return 0


def cmd_show(a) -> int:
    s = load(a.name)
    print(s.f.read_text(encoding="utf-8"), end="")
    for p in sorted(s.dir.rglob("*")):
        if p.is_file() and p.name != "set.yaml":
            print(f"  {sha12(p)}  {p.relative_to(s.dir)}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    sub.add_parser("list").set_defaults(fn=cmd_list)
    p = sub.add_parser("show"); p.add_argument("name"); p.set_defaults(fn=cmd_show)
    p = sub.add_parser("new"); p.add_argument("name"); p.add_argument("--from", dest="src")
    p.add_argument("--poses-dir"); p.add_argument("--scene"); p.add_argument("--mirror", choices=["true", "false"])
    p.set_defaults(fn=cmd_new)
    p = sub.add_parser("sync"); p.add_argument("name"); p.add_argument("--scene"); p.set_defaults(fn=cmd_sync)
    p = sub.add_parser("check"); p.add_argument("name"); p.add_argument("--mode", required=True, choices=["record", "rollout", "teleop"])
    p.add_argument("--scene"); p.set_defaults(fn=cmd_check)
    p = sub.add_parser("freeze"); p.add_argument("name"); p.add_argument("--from-run", required=True); p.set_defaults(fn=cmd_freeze)
    p = sub.add_parser("set-model"); p.add_argument("name"); p.add_argument("--model-dir", required=True)
    p.add_argument("--model-hf"); p.add_argument("--lang"); p.set_defaults(fn=cmd_set_model)
    p = sub.add_parser("restore-calib"); p.add_argument("name"); p.set_defaults(fn=cmd_restore_calib)
    p = sub.add_parser("current"); p.add_argument("name", nargs="?"); p.set_defaults(fn=cmd_current)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
