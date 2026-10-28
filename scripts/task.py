#!/usr/bin/env python3
"""タスクの台帳（config/tasks/<タスク ID>.yaml）を読む。標準ライブラリだけで動く（どの python でもよい）。

タスク = 1 つの仕事（指示文は 1 つ）。台帳に置くもの:
  タスク ID・題名・指示文（lang）・タスクファイル（env_file）・カメラ
  課題リストと台帳の文書の場所（issue_list）と、このタスクの課題コード（issues。録る・録らないは台帳の状態で持つ）
  条件の版（v<N>.pose_set / v<N>.scene）と今の版（current_version）・名前の型
命名則の正本は Orboh/doma-sorting-dev 07-collection/README.md「命名則」（2026-10-28）:
  外に見せる ID   <タスク ID>_<課題コードを小文字>            例 takeei_wood_t1-l2
  データセット    Orboh/bi_openarm_<外に見せる ID>_<カメラ>_v<条件の版>
  モデル          Orboh/gr00t-bi-openarm-<外に見せる ID をハイフンに>-v<N>（混ぜたら <タスク>-mix）

使い方（~/projects/openarm-orboh）:
  python3 scripts/task.py list [--json]                                # タスクと課題コード・外に見せる ID
  python3 scripts/task.py check <task> [--issue T1-L2]                 # 収集の前の突き合わせ（食い違えば終了コード 1）
  python3 scripts/task.py name <task> <issue> [--model-version N] [--mix] [--json]

check が見るもの:
  タスクファイルの TASK・TASK_SLUG・POSE_SET・SCENE・カメラが台帳（今の条件の版）と一致する
  開始姿勢セットのシーンと scene_sha が今のシーン YAML と一致する（sha は pose_set.py と同じ）・凍結済みか
  シーン YAML に TODO が残っていない（collect_manifest.py check-scene と同じ判定）
  --issue の課題コードがこのタスクの一覧にある（大文字・小文字は問わず、台帳の書き方 T1-L0a などにそろえる）
  --issue があり、課題リストと台帳の文書が手元にあれば、その課題コードの節がある（無ければ警告だけ）
環境変数:
  TASK_REPO        台帳・タスクファイル・シーンを読む repo（既定はこの repo。テスト用）
  PS_ROOT          開始姿勢セットの置き場（既定 <TASK_REPO>/config/pose_sets・pose_set.py と同じ）
  TASK_LEDGER_DIR  課題リストと台帳の repo の手元の clone（既定 ~/projects/<repo 名>。例 ~/projects/doma-sorting-dev）
"""
from __future__ import annotations

import argparse
import json
import os
import re
import shlex
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from collect_manifest import count_scene_todos  # noqa: E402  シーンの TODO の判定を check-scene とそろえる
from pose_set import sha12  # noqa: E402  scene_sha の計算を pose_set.py とそろえる

REPO = Path(os.environ.get("TASK_REPO") or Path(__file__).resolve().parent.parent)
TASKS = REPO / "config/tasks"
PS_ROOT = Path(os.environ.get("PS_ROOT") or REPO / "config/pose_sets")
ENV_KEYS = ("TASK", "TASK_SLUG", "POSE_SET", "SCENE", "WRIST_RIGHT", "WRIST_LEFT")  # check が見るタスクファイルの値
ISSUE_RE = re.compile(r"^T\d+(-L\d+[a-z]?)?$", re.I)  # T2・T1-L2・T1-L0a


def read_kv(p: Path) -> dict[str, str]:
    """1 行 1 キー・入れ子なしの YAML（set.yaml と同じ読み方）。"""
    kv: dict[str, str] = {}
    for line in p.read_text(encoding="utf-8").splitlines():
        if line.lstrip().startswith("#") or ":" not in line:
            continue
        k, v = line.split(":", 1)
        kv[k.strip()] = v.strip()
    return kv


def read_env(p: Path) -> dict[str, str]:
    """タスクファイル（KEY=値 の行）を bash の source と同じ形で読む。引用符と行末コメントを外す。"""
    env: dict[str, str] = {}
    for line in p.read_text(encoding="utf-8").splitlines():
        m = re.match(r"^\s*(?:export\s+)?([A-Z_][A-Z0-9_]*)=(.*)$", line)
        if not m:
            continue
        try:
            env[m.group(1)] = " ".join(shlex.split(m.group(2), comments=True))
        except ValueError:
            env[m.group(1)] = m.group(2).strip()
    return env


def env_cameras(env: dict[str, str]) -> str:
    """タスクファイルのカメラ設定 → データセット名のカメラ部分（collect_bimanual.sh の OBS と同じ組み立て）。"""
    obs = "chest"
    if env.get("WRIST_RIGHT", "true") == "true":
        obs += "_wristr"
    if env.get("WRIST_LEFT", "false") == "true":
        obs += "_wristl"
    return obs


class Task:
    def __init__(self, name: str):
        self.name = name
        self.f = TASKS / f"{name}.yaml"
        self.kv = read_kv(self.f) if self.f.exists() else {}

    def get(self, k: str) -> str:
        return self.kv.get(k, "")

    @property
    def issues(self) -> list[str]:
        return [x.strip() for x in self.get("issues").split(",") if x.strip()]

    def canon(self, code: str) -> str:
        """手で打った課題コード → 台帳の書き方（大文字・小文字を問わない。t1-l0a → T1-L0a）。一覧に無ければそのまま。"""
        code = code.strip()
        return next((i for i in self.issues if i.lower() == code.lower()), code)

    @property
    def version(self) -> str:
        return self.get("current_version")

    def versions(self) -> list[str]:
        vs = {k.split(".", 1)[0] for k in self.kv if re.match(r"^v\d+\.", k)}
        return sorted(vs, key=lambda v: int(v[1:]))

    def cond(self, v: str | None = None) -> dict[str, str]:
        v = v or self.version
        return {"version": v, "pose_set": self.get(f"{v}.pose_set"), "scene": self.get(f"{v}.scene"), "note": self.get(f"{v}.note")}

    def ext_id(self, code: str) -> str:
        return f"{self.name}_{code.strip().lower()}"

    def fill(self, key: str, code: str, n: int = 1) -> str:
        eid = self.ext_id(code)
        return self.get(key).format(id=eid, id_hyphen=eid.replace("_", "-"), task_hyphen=self.name.replace("_", "-"),
                                    cameras=self.get("cameras"), version=self.version, n=n)

    def ledger_path(self) -> Path | None:
        """issue_list（<owner>/<repo>:<repo 内のパス>）→ 手元の clone の中のファイル。"""
        ref = self.get("issue_list")
        if ":" not in ref:
            return None
        repo, rel = ref.split(":", 1)
        base = os.environ.get("TASK_LEDGER_DIR") or str(Path.home() / "projects" / repo.rsplit("/", 1)[-1])
        return Path(base).expanduser() / rel


def task_names() -> list[str]:
    return sorted(p.stem for p in TASKS.glob("*.yaml"))


def load(name: str) -> Task:
    t = Task(name)
    if not t.f.exists():
        sys.exit(f"[task] タスクが台帳に無い: {t.f}（候補: {' '.join(task_names()) or 'なし'}）")
    return t


def ledger_has_section(text: str, code: str) -> bool:
    """課題リストと台帳に、見出しが課題コードで始まる節があるか（例「### T1-L2（`takeei_wood_t1-l2`）」）。"""
    pat = re.compile(rf"^#{{2,6}}\s+{re.escape(code)}(?![-\w])", re.M)
    return bool(pat.search(text))


# ---- list -----------------------------------------------------------------------
def task_info(t: Task) -> dict:
    c = t.cond()
    return {"task": t.name, "title": t.get("title"), "lang": t.get("lang"), "env_file": t.get("env_file"),
            "cameras": t.get("cameras"), "issue_list": t.get("issue_list"), "current_version": c["version"],
            "pose_set": c["pose_set"], "scene": c["scene"],
            "issues": [{"code": i, "id": t.ext_id(i), "dataset": t.fill("dataset_name", i)} for i in t.issues]}


def cmd_list(a) -> int:
    infos = [task_info(Task(n)) for n in task_names()]
    if a.json:
        print(json.dumps({"tasks": infos}, ensure_ascii=False, indent=2))
        return 0
    for x in infos:
        print(f"{x['task']}  {x['title']}")
        print(f"  指示文: {x['lang']}")
        print(f"  今の版: {x['current_version']}（開始姿勢 {x['pose_set']}・シーン {x['scene']}）  タスクファイル: {x['env_file']}")
        for i in x["issues"]:
            print(f"  {i['code']:8s} {i['id']:28s} {i['dataset']}")
    return 0


# ---- check ----------------------------------------------------------------------
def cmd_check(a) -> int:
    t = load(a.name)
    c = t.cond()
    bad: list[str] = []   # 止める食い違い
    warn: list[str] = []  # 止めない注意
    if not c["version"] or not c["pose_set"] or not c["scene"]:
        bad.append(f"台帳の今の条件の版が書けていない（current_version={c['version'] or '空'}・"
                   f"{c['version']}.pose_set={c['pose_set'] or '空'}・{c['version']}.scene={c['scene'] or '空'}）")

    # タスクファイル
    env_file = Path(a.env_file).expanduser() if a.env_file else REPO / t.get("env_file")
    if a.env_file and env_file.resolve() != (REPO / t.get("env_file")).resolve():
        warn.append(f"台帳のタスクファイルは {t.get('env_file')}。いま見ているのは {env_file}")
    if not env_file.is_file():
        bad.append(f"タスクファイルが無い: {env_file}")
    else:
        env = read_env(env_file)
        if a.environ:  # collect_bimanual.sh から: 呼び出し側で上書きした値（POSE_SET=... など）を含めて見る
            env.update({k: os.environ[k] for k in ENV_KEYS if k in os.environ})
        for key, want, what in (("TASK", t.get("lang"), "指示文"), ("TASK_SLUG", t.name, "タスク ID"),
                                ("POSE_SET", c["pose_set"], f"今の版 {c['version']} の開始姿勢セット"),
                                ("SCENE", c["scene"], f"今の版 {c['version']} のシーン")):
            if env.get(key, "") != want:
                bad.append(f"{key} が{what}と違う（タスクファイル '{env.get(key, '')}' / 台帳 '{want}'）")
        if t.get("cameras") and env_cameras(env) != t.get("cameras"):
            bad.append(f"カメラが違う（タスクファイルの WRIST_RIGHT / WRIST_LEFT からは {env_cameras(env)} / 台帳 {t.get('cameras')}）")

    # 開始姿勢セットとシーン
    scene_path = REPO / "scenes" / f"{c['scene']}.yaml"
    if c["scene"] and not scene_path.is_file():
        bad.append(f"シーンが無い: {scene_path}")
    set_f = PS_ROOT / c["pose_set"] / "set.yaml"
    if c["pose_set"] and not set_f.is_file():
        bad.append(f"開始姿勢セットが無い: {set_f}")
    elif c["pose_set"]:
        s = read_kv(set_f)
        status = s.get("status") or "frozen"  # pose_set.py と同じ: status の無い旧セットは凍結扱い
        print(f"[task] 開始姿勢セット {c['pose_set']}: {status}" + (f"（{s.get('frozen_at')} に凍結）" if s.get("frozen_at") else ""))
        mism: list[str] = []
        if s.get("scene") != c["scene"]:
            mism.append(f"セットのシーンが {s.get('scene') or '空'}（台帳の今の版は {c['scene']}）")
        if scene_path.is_file() and s.get("scene_sha") != sha12(scene_path):
            mism.append(f"セットの scene_sha が今の scenes/{c['scene']}.yaml と違う（セット {s.get('scene_sha') or '空'} / 今 {sha12(scene_path)}）")
        if s.get("lang") and s.get("lang") != t.get("lang"):
            mism.append(f"セットの指示文 '{s.get('lang')}' が台帳の指示文と違う")
        if status == "draft":  # draft は収集の開始で今のシーンを取り込み直すので止めない
            warn += [f"{m}（draft なので収集の開始で取り込み直す）" for m in mism]
        else:
            bad += mism

    # シーンの TODO
    if scene_path.is_file():
        n = count_scene_todos(scene_path)
        if n is None:
            warn.append(f"scenes/{c['scene']}.yaml の TODO を数えられなかった")
        elif n:
            bad.append(f"scenes/{c['scene']}.yaml に TODO が {n} 件残っている")

    # 課題コードと台帳の文書
    code = t.canon(a.issue) if a.issue else ""
    if code and code not in t.issues:
        bad.append(f"課題コード {code} はこのタスクの課題コードに無い（一覧: {', '.join(t.issues)}）")
    elif code:
        lp = t.ledger_path()
        if lp is None:
            warn.append("台帳に issue_list が無い（課題リストと台帳の文書の節を確かめられない）")
        elif not lp.is_file():
            warn.append(f"課題リストと台帳の文書が手元に無いので節を確かめていない: {lp}（場所は TASK_LEDGER_DIR で変えられる）")
        elif not ledger_has_section(lp.read_text(encoding="utf-8"), code):
            warn.append(f"課題リストと台帳 {lp.name} に {code} の節がまだ無い（録ったら台帳に節を足す）")

    for w in warn:
        print(f"[task] 注意: {w}", file=sys.stderr)
    if bad:
        for b in bad:
            print(f"[task] {t.name}: {b}", file=sys.stderr)
        print(f"[task] 止める。タスクファイル・開始姿勢セット・シーンを config/tasks/{t.name}.yaml の今の版（{c['version']}）にそろえてから録る。"
              f"版を上げたなら台帳の current_version と v<N>.pose_set / v<N>.scene を直す", file=sys.stderr)
        return 1
    print(f"[task] {t.name}: 今の版 {c['version']}（開始姿勢 {c['pose_set']}・シーン {c['scene']}）とタスクファイルが一致")
    if code:
        print(f"[task] {t.ext_id(code)} → {t.fill('dataset_name', code)}")
    return 0


# ---- name -----------------------------------------------------------------------
def cmd_name(a) -> int:
    t = load(a.name)
    code = t.canon(a.issue)
    if not ISSUE_RE.match(code):
        sys.exit(f"[task] 課題コードの形が違う: {a.issue}（例 T1-L2・T1-L0a・T2）")
    if code not in t.issues:
        sys.exit(f"[task] 課題コード {code} は {t.name} の課題コードに無い（一覧: {', '.join(t.issues)}）")
    out = {"task": t.name, "issue": code, "id": t.ext_id(code), "version": t.version,
           "dataset": t.fill("dataset_name", code),
           "model": t.fill("model_mix_name" if a.mix else "model_name", code, a.model_version)}
    if a.json:
        print(json.dumps(out, ensure_ascii=False, indent=2))
        return 0
    print(f"外に見せる ID: {out['id']}")
    print(f"データセット: {out['dataset']}（条件の版 {out['version']}）")
    print(f"モデル: {out['model']}" + ("（複数の課題コードを混ぜたもの）" if a.mix else ""))
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    p = sub.add_parser("list"); p.add_argument("--json", action="store_true"); p.set_defaults(fn=cmd_list)
    p = sub.add_parser("check"); p.add_argument("name"); p.add_argument("--issue")
    p.add_argument("--env-file", help="台帳の env_file の代わりに見るタスクファイル（collect_bimanual.sh は今の ENV_FILE を渡す）")
    p.add_argument("--environ", action="store_true", help="環境変数にある TASK・POSE_SET などで上書きして見る（collect_bimanual.sh から）")
    p.set_defaults(fn=cmd_check)
    p = sub.add_parser("name"); p.add_argument("name"); p.add_argument("issue")
    p.add_argument("--model-version", type=int, default=1); p.add_argument("--mix", action="store_true")
    p.add_argument("--json", action="store_true"); p.set_defaults(fn=cmd_name)
    a = ap.parse_args()
    return a.fn(a)


if __name__ == "__main__":
    sys.exit(main())
