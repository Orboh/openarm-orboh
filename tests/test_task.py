"""scripts/task.py（タスクの台帳）と collect_bimanual.sh からの呼び出しのテスト。実機なし・標準ライブラリだけ。

  python3 -m unittest discover -s tests -v      # repo の直下で

一時フォルダに小さな repo（台帳・タスクファイル・開始姿勢セット・シーン・課題リストと台帳）を作り、
TASK_REPO / PS_ROOT / TASK_LEDGER_DIR でそこを見せて task.py を外から呼ぶ。
collect_bimanual.sh は一時の repo に写し、can_check.sh を「印を出して止まる」偽物に替えて回す
（CAN が UP の収集機でテストを回しても record まで進まないように）。
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
TASK_PY = REPO / "scripts/task.py"

TASK_YAML = """# テスト用
task: wood
title: テストの木材
lang: Pick up the wood and place it in the box
env_file: config/bimanual-collect.wood.env
cameras: chest_wristr_wristl
issue_list: Orboh/doma-sorting-dev:07-collection/wood-issue-list.md
ledger: Orboh/doma-sorting-dev:07-collection/wood-ledger.md
issues: T1-L0a,T1-L1,T1-L2,T2,T3-L1
current_version: v2
v1.pose_set: old-set
v1.scene: desk-v1
v1.note: もう録らない
v2.pose_set: new-set
v2.scene: desk-v1
v2.note: 今の版
dataset_name: Orboh/bi_openarm_{id}_{cameras}_{version}
model_name: Orboh/gr00t-bi-openarm-{id_hyphen}-v{n}
model_mix_name: Orboh/gr00t-bi-openarm-{task_hyphen}-mix-v{n}
"""

ENV = """# テスト用のタスクファイル
TASK="Pick up the wood and place it in the box"
TASK_SLUG=wood
POSE_SET=new-set
SCENE=desk-v1   # 行末コメント
OPERATOR=
WRIST_RIGHT=true
WRIST_LEFT=true
"""

SCENE = """# シーン（TODO はコメントの中なら数えない）
name: desk-v1
table:
  height_mm: 900
"""

LEDGER = """# 課題リストと台帳
## 3. 台帳
### T1-L1（`wood_t1-l1`）
### T1-L2（`wood_t1-l2`）
"""


def sha12(text: str) -> str:
    return hashlib.sha256(text.encode("utf-8")).hexdigest()[:12]


class Fixture(unittest.TestCase):
    def setUp(self):
        self.tmp = Path(tempfile.mkdtemp(prefix="task-test-"))
        self.repo = self.tmp / "repo"
        self.ledger = self.tmp / "doma-sorting-dev"
        self.write("config/tasks/wood.yaml", TASK_YAML)
        self.write("config/bimanual-collect.wood.env", ENV)
        self.write("scenes/desk-v1.yaml", SCENE)
        self.write_set("new-set", "frozen", "desk-v1", sha12(SCENE))
        self.write_set("old-set", "frozen", "desk-v1", sha12(SCENE))
        (self.ledger / "07-collection").mkdir(parents=True)
        (self.ledger / "07-collection/wood-ledger.md").write_text(LEDGER, encoding="utf-8")

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def write(self, rel: str, text: str) -> Path:
        p = self.repo / rel
        p.parent.mkdir(parents=True, exist_ok=True)
        p.write_text(text, encoding="utf-8")
        return p

    def write_set(self, name: str, status: str, scene: str, scene_sha: str, extra: str = "") -> None:
        self.write(f"config/pose_sets/{name}/set.yaml",
                   f"# 開始姿勢セット\nname: {name}\nstatus: {status}\nscene: {scene}\nscene_sha: {scene_sha}\n{extra}")

    def env(self, **kw) -> dict:
        e = {k: v for k, v in os.environ.items() if k not in ("PS_ROOT", "TASK", "TASK_SLUG", "POSE_SET", "SCENE",
                                                                "WRIST_RIGHT", "WRIST_LEFT", "ISSUE", "ENV_FILE")}
        e.update(TASK_REPO=str(self.repo), TASK_LEDGER_DIR=str(self.ledger))
        e.update(kw)
        return e

    def task(self, *args, **kw) -> subprocess.CompletedProcess:
        return subprocess.run([sys.executable, str(TASK_PY), *args], capture_output=True, text=True, env=self.env(**kw))


class TestList(Fixture):
    def test_human(self):
        r = self.task("list")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("wood_t1-l2", r.stdout)
        self.assertIn("Orboh/bi_openarm_wood_t2_chest_wristr_wristl_v2", r.stdout)

    def test_json(self):
        r = self.task("list", "--json")
        self.assertEqual(r.returncode, 0, r.stderr)
        t = json.loads(r.stdout)["tasks"][0]
        self.assertEqual(t["task"], "wood")
        self.assertEqual(t["current_version"], "v2")
        self.assertEqual(t["pose_set"], "new-set")
        self.assertEqual([i["code"] for i in t["issues"]], ["T1-L0a", "T1-L1", "T1-L2", "T2", "T3-L1"])
        self.assertEqual(t["issues"][0]["id"], "wood_t1-l0a")
        self.assertEqual(t["issues"][2]["id"], "wood_t1-l2")


class TestCheck(Fixture):
    def test_match(self):
        r = self.task("check", "wood", "--issue", "T1-L2")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("frozen", r.stdout)
        self.assertIn("Orboh/bi_openarm_wood_t1-l2_chest_wristr_wristl_v2", r.stdout)
        self.assertNotIn("注意", r.stderr)

    def test_issue_lowercase_ok(self):
        self.assertEqual(self.task("check", "wood", "--issue", "t1-l1").returncode, 0)

    def test_issue_l0a_any_case(self):  # 台帳の書き方 T1-L0a にそろえる（T1-L0A にしない）
        r = self.task("check", "wood", "--issue", "T1-L0A")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("wood_t1-l0a", r.stdout)
        self.assertIn("T1-L0a の節がまだ無い", r.stderr)  # 台帳の文書に節が無い → 警告だけ

    def test_pose_set_mismatch(self):  # タスクファイルが旧版の開始姿勢セットのまま
        self.write("config/bimanual-collect.wood.env", ENV.replace("POSE_SET=new-set", "POSE_SET=old-set"))
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 1)
        self.assertIn("POSE_SET", r.stderr)

    def test_lang_and_slug_mismatch(self):
        self.write("config/bimanual-collect.wood.env",
                   ENV.replace("TASK_SLUG=wood", "TASK_SLUG=wood_l1").replace("the box", "the bin"))
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 1)
        self.assertIn("TASK が指示文と違う", r.stderr)
        self.assertIn("TASK_SLUG", r.stderr)

    def test_camera_mismatch(self):
        self.write("config/bimanual-collect.wood.env", ENV.replace("WRIST_LEFT=true", "WRIST_LEFT=false"))
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 1)
        self.assertIn("chest_wristr", r.stderr)

    def test_scene_changed_after_freeze(self):  # シーン YAML を凍結後に書き換えた
        self.write("scenes/desk-v1.yaml", SCENE + "mat:\n  present: true\n")
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 1)
        self.assertIn("scene_sha", r.stderr)

    def test_set_scene_differs(self):
        self.write_set("new-set", "frozen", "other-scene", sha12(SCENE))
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 1)
        self.assertIn("セットのシーンが other-scene", r.stderr)

    def test_draft_set_only_warns(self):  # draft は収集の開始で取り込み直すので止めない
        self.write_set("new-set", "draft", "desk-v1", "")
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("draft", r.stdout)
        self.assertIn("取り込み直す", r.stderr)

    def test_set_lang_differs(self):
        self.write_set("new-set", "frozen", "desk-v1", sha12(SCENE), "lang: Pick up the bottle\n")
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 1)
        self.assertIn("セットの指示文", r.stderr)

    def test_pose_set_missing(self):
        shutil.rmtree(self.repo / "config/pose_sets/new-set")
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 1)
        self.assertIn("開始姿勢セットが無い", r.stderr)

    def test_scene_todo(self):
        scene = SCENE + "lighting: TODO\n"
        self.write("scenes/desk-v1.yaml", scene)
        self.write_set("new-set", "frozen", "desk-v1", sha12(scene))
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 1)
        self.assertIn("TODO が 1 件", r.stderr)

    def test_issue_not_listed(self):
        r = self.task("check", "wood", "--issue", "T9")
        self.assertEqual(r.returncode, 1)
        self.assertIn("T9 はこのタスクの課題コードに無い", r.stderr)

    def test_no_issue_skips_ledger(self):  # --issue なしでは台帳の文書の節を見ない（節の無いコードで警告を並べない）
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn("節", r.stderr)

    def test_ledger_missing_warns_only(self):
        r = self.task("check", "wood", "--issue", "T1-L1", TASK_LEDGER_DIR=str(self.tmp / "nowhere"))
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("手元に無い", r.stderr)

    def test_ledger_section_missing_warns_only(self):  # T2 は一覧にあるが台帳の文書に節が無い
        r = self.task("check", "wood", "--issue", "T2")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn("T2 の節がまだ無い", r.stderr)

    def test_ledger_falls_back_to_issue_list(self):  # ledger の無いタスクは issue_list の文書を台帳として見る（2026-10-31 より前の形）
        self.write("config/tasks/wood.yaml", TASK_YAML.replace("ledger: Orboh/doma-sorting-dev:07-collection/wood-ledger.md\n", ""))
        (self.ledger / "07-collection/wood-ledger.md").rename(self.ledger / "07-collection/wood-issue-list.md")
        r = self.task("check", "wood", "--issue", "T1-L1")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertNotIn("節", r.stderr)

    def test_env_file_missing(self):
        (self.repo / "config/bimanual-collect.wood.env").unlink()
        r = self.task("check", "wood")
        self.assertEqual(r.returncode, 1)
        self.assertIn("タスクファイルが無い", r.stderr)

    def test_environ_override(self):  # collect_bimanual.sh で呼び出し側が POSE_SET を上書きした
        r = self.task("check", "wood", "--environ", POSE_SET="old-set")
        self.assertEqual(r.returncode, 1)
        self.assertIn("POSE_SET", r.stderr)
        self.assertEqual(self.task("check", "wood", POSE_SET="old-set").returncode, 0)  # --environ なしは見ない

    def test_unknown_task(self):
        r = self.task("check", "nope")
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("台帳に無い", r.stderr)


class TestName(Fixture):
    def test_name(self):
        r = self.task("name", "wood", "T1-L2", "--json")
        self.assertEqual(r.returncode, 0, r.stderr)
        out = json.loads(r.stdout)
        self.assertEqual(out["id"], "wood_t1-l2")
        self.assertEqual(out["dataset"], "Orboh/bi_openarm_wood_t1-l2_chest_wristr_wristl_v2")
        self.assertEqual(out["model"], "Orboh/gr00t-bi-openarm-wood-t1-l2-v1")

    def test_mix_and_version(self):
        out = json.loads(self.task("name", "wood", "t2", "--mix", "--model-version", "3", "--json").stdout)
        self.assertEqual(out["model"], "Orboh/gr00t-bi-openarm-wood-mix-v3")
        self.assertEqual(out["id"], "wood_t2")

    def test_l0a(self):
        out = json.loads(self.task("name", "wood", "t1-l0a", "--json").stdout)
        self.assertEqual(out["issue"], "T1-L0a")
        self.assertEqual(out["dataset"], "Orboh/bi_openarm_wood_t1-l0a_chest_wristr_wristl_v2")
        self.assertEqual(out["model"], "Orboh/gr00t-bi-openarm-wood-t1-l0a-v1")

    def test_not_listed(self):
        r = self.task("name", "wood", "T1-L3")
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("T1-L3 は wood の課題コードに無い", r.stderr)


class TestRealRegistry(unittest.TestCase):
    """repo に入れた本物の台帳（config/tasks/）が、タスクファイル・開始姿勢セット・シーンとそろっているか。"""

    def run_real(self, *args):
        e = {k: v for k, v in os.environ.items() if k not in ("TASK_REPO", "PS_ROOT")}
        e["TASK_LEDGER_DIR"] = "/nonexistent-ledger"  # 手元の clone の有無で結果を変えない
        return subprocess.run([sys.executable, str(TASK_PY), *args], capture_output=True, text=True, env=e)

    def test_takeei_wood(self):
        r = self.run_real("check", "takeei_wood", "--issue", "T1-L1")
        self.assertEqual(r.returncode, 0, r.stderr)
        out = json.loads(self.run_real("name", "takeei_wood", "T1-L2", "--json").stdout)
        self.assertEqual(out["dataset"], "Orboh/bi_openarm_takeei_wood_t1-l2_chest_wristr_wristl_v2")
        self.assertEqual(out["model"], "Orboh/gr00t-bi-openarm-takeei-wood-t1-l2-v1")
        codes = [i["code"] for i in json.loads(self.run_real("list", "--json").stdout)["tasks"][0]["issues"]]
        self.assertEqual(codes, ["T1-L0a", "T1-L1", "T1-L2", "T1-L3", "T2", "T3-L1", "T3-L2", "T3-L3",
                                 "T4", "T5-L1", "T5-L2", "T5-L3"])


class TestCollectHook(Fixture):
    """collect_bimanual.sh: 台帳のあるタスクだけ gate/session の前に check を呼ぶ。dry と台帳の無いタスクは呼ばない。"""

    def setUp(self):
        super().setUp()
        for f in ("collect_bimanual.sh", "task.py", "pose_set.py", "collect_manifest.py"):
            self.write(f"scripts/{f}", (REPO / "scripts" / f).read_text(encoding="utf-8"))
        self.write("scripts/can_check.sh", 'echo "CAN_CHECK_REACHED"; exit 1\n')  # ここまで来たら止める

    def collect(self, mode: str, env_file: str, **kw) -> subprocess.CompletedProcess:
        e = self.env(ENV_FILE=str(self.repo / env_file), **kw)
        e.pop("TASK_REPO")  # 写した collect_bimanual.sh は自分の場所から repo を決める（task.py も同じ）
        return subprocess.run(["bash", str(self.repo / "scripts/collect_bimanual.sh"), mode],
                              capture_output=True, text=True, env=e)

    def test_gate_stops_on_mismatch(self):
        self.write("config/bimanual-collect.wood.env", ENV.replace("POSE_SET=new-set", "POSE_SET=old-set"))
        r = self.collect("gate", "config/bimanual-collect.wood.env")
        self.assertEqual(r.returncode, 1)
        self.assertIn("[task]", r.stderr)
        self.assertNotIn("CAN_CHECK_REACHED", r.stdout)

    def test_gate_passes_to_can_check(self):
        r = self.collect("session", "config/bimanual-collect.wood.env", ISSUE="T1-L1")
        self.assertIn("[task] wood: 今の版 v2", r.stdout)
        self.assertIn("CAN_CHECK_REACHED", r.stdout)

    def test_gate_stops_on_issue(self):
        r = self.collect("gate", "config/bimanual-collect.wood.env", ISSUE="T9")
        self.assertEqual(r.returncode, 1)
        self.assertNotIn("CAN_CHECK_REACHED", r.stdout)

    def test_dry_skips(self):
        self.write("config/bimanual-collect.wood.env", ENV.replace("POSE_SET=new-set", "POSE_SET=old-set"))
        r = self.collect("dry", "config/bimanual-collect.wood.env")
        self.assertNotIn("[task]", r.stdout + r.stderr)
        self.assertIn("CAN_CHECK_REACHED", r.stdout)

    def test_task_without_registry_skips(self):  # ボトルなど台帳の無いタスク
        self.write("config/bimanual-collect.bottle.env", ENV.replace("TASK_SLUG=wood", "TASK_SLUG=bottle"))
        r = self.collect("gate", "config/bimanual-collect.bottle.env")
        self.assertNotIn("[task]", r.stdout + r.stderr)
        self.assertIn("CAN_CHECK_REACHED", r.stdout)


if __name__ == "__main__":
    unittest.main()
