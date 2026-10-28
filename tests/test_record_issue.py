"""課題コード（ISSUE）を収集の記録に入れる変更のテスト（2026-10-28）。実機なし・標準ライブラリだけ。

  python3 -m unittest discover -s tests -v      # repo の直下で

  - scripts/task.py issue: ISSUE とタグから課題コードを決める（一覧に無い・どちらも無い・食い違いは止める）
  - collect_bimanual.sh: 台帳のあるタスクの gate / session は課題コードが必須（dry と台帳の無いタスクは今までどおり）
  - record_bimanual_openarm.sh: データセットの名前（台帳あり・なし・再開）と manifest の task_ledger・command.sh。
    lerobot-record は引数を出すだけの偽物に替え、一時の repo で最後まで回す（CAN・カメラには触らない）
台帳・タスクファイル・開始姿勢セット・シーンの雛形は tests/test_task.py の Fixture を使う。
"""
from __future__ import annotations

import json
import os
import subprocess
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_task import ENV, REPO, Fixture  # noqa: E402

BASE = "Orboh/bi_openarm_wood_t1-l2_chest_wristr_wristl_v2"  # 台帳 wood・課題コード T1-L2・今の版 v2 の名前（日時の前まで）


class TestIssue(Fixture):
    """task.py issue: 課題コードの決め方。"""

    def issue(self, *args):
        return self.task("issue", "wood", *args)

    def test_from_issue(self):
        r = self.issue("--issue", "t1-l2")
        self.assertEqual((r.returncode, r.stdout.strip()), (0, "T1-L2"), r.stderr)

    def test_from_tag(self):
        for tag, want in (("t1-l2-s1", "T1-L2"), ("t2-s1", "T2"), ("t1-l0a-s1", "T1-L0a"), ("T1-L1-s3", "T1-L1")):
            with self.subTest(tag=tag):
                r = self.issue("--tag", tag)
                self.assertEqual((r.returncode, r.stdout.strip()), (0, want), r.stderr)

    def test_issue_wins_when_tag_has_none(self):
        r = self.issue("--issue", "T2", "--tag", "s4")
        self.assertEqual((r.returncode, r.stdout.strip()), (0, "T2"), r.stderr)

    def test_issue_and_tag_agree(self):
        self.assertEqual(self.issue("--issue", "T1-L2", "--tag", "t1-l2-s1").stdout.strip(), "T1-L2")

    def test_neither(self):
        r = self.issue("--tag", "s1")
        self.assertEqual(r.returncode, 1)
        self.assertEqual(r.stdout, "")
        self.assertIn("課題コードが要る", r.stderr)

    def test_not_listed_issue(self):
        r = self.issue("--issue", "T9")
        self.assertEqual(r.returncode, 1)
        self.assertIn("T9 は wood の課題コードに無い", r.stderr)

    def test_not_listed_tag(self):  # T1-L3 は wood の一覧に無い（T1 だけを拾い直さない）
        r = self.issue("--tag", "t1-l3-s1")
        self.assertEqual(r.returncode, 1)
        self.assertIn("t1-l3 は wood の課題コードに無い", r.stderr)

    def test_conflict(self):
        r = self.issue("--issue", "T2", "--tag", "t1-l2-s1")
        self.assertEqual(r.returncode, 1)
        self.assertIn("違う", r.stderr)

    def test_two_codes_in_tag(self):
        r = self.issue("--tag", "t1-l2-t2-s1")
        self.assertEqual(r.returncode, 1)
        self.assertIn("2 つ以上", r.stderr)

    def test_bad_shape(self):
        self.assertEqual(self.issue("--issue", "L2").returncode, 1)


class TestNameFields(Fixture):
    def test_fields(self):
        r = self.task("name", "wood", "t1-l0a", "--fields", "task,issue,id,version,dataset")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertEqual(r.stdout.split(),
                         ["wood", "T1-L0a", "wood_t1-l0a", "v2", "Orboh/bi_openarm_wood_t1-l0a_chest_wristr_wristl_v2"])

    def test_unknown_field(self):
        r = self.task("name", "wood", "T2", "--fields", "dataset,nope")
        self.assertNotEqual(r.returncode, 0)
        self.assertIn("nope", r.stderr)


class TestCollectIssue(Fixture):
    """collect_bimanual.sh: 課題コードの必須化。can_check.sh は「印を出して止まる」偽物（test_task.py と同じ）。"""

    def setUp(self):
        super().setUp()
        for f in ("collect_bimanual.sh", "task.py", "pose_set.py", "collect_manifest.py"):
            self.write(f"scripts/{f}", (REPO / "scripts" / f).read_text(encoding="utf-8"))
        self.write("scripts/can_check.sh", 'echo "CAN_CHECK_REACHED"; exit 1\n')

    def collect(self, *args, env_file: str = "config/bimanual-collect.wood.env", **kw) -> subprocess.CompletedProcess:
        e = self.env(ENV_FILE=str(self.repo / env_file), **kw)
        e.pop("TASK_REPO")
        e.pop("CHECK_ONLY", None)
        e.update({k: v for k, v in kw.items()})
        return subprocess.run(["bash", str(self.repo / "scripts/collect_bimanual.sh"), *args],
                              capture_output=True, text=True, env=e)

    def test_gate_needs_issue(self):
        r = self.collect("gate")
        self.assertEqual(r.returncode, 1)
        self.assertIn("課題コードが要る", r.stderr)
        self.assertNotIn("CAN_CHECK_REACHED", r.stdout)

    def test_gate_with_issue(self):
        r = self.collect("gate", ISSUE="t1-l2")
        self.assertIn(f"[task] wood_t1-l2 → {BASE}", r.stdout)
        self.assertIn("CAN_CHECK_REACHED", r.stdout)

    def test_session_issue_from_tag(self):
        for tag, eid in (("t1-l2-s1", "wood_t1-l2"), ("t2-s1", "wood_t2"), ("t1-l0a-s1", "wood_t1-l0a")):
            with self.subTest(tag=tag):
                r = self.collect("session", tag)
                self.assertIn(f"[task] {eid} → Orboh/bi_openarm_{eid}_chest_wristr_wristl_v2", r.stdout)
                self.assertIn("CAN_CHECK_REACHED", r.stdout)

    def test_session_tag_without_issue(self):
        r = self.collect("session", "s1")
        self.assertEqual(r.returncode, 1)
        self.assertIn("課題コードが要る", r.stderr)
        self.assertNotIn("CAN_CHECK_REACHED", r.stdout)

    def test_session_tag_not_listed(self):
        r = self.collect("session", "t9-s1")
        self.assertEqual(r.returncode, 1)
        self.assertIn("課題コードに無い", r.stderr)
        self.assertNotIn("CAN_CHECK_REACHED", r.stdout)

    def test_issue_and_tag_conflict(self):
        r = self.collect("session", "t1-l2-s1", ISSUE="T2")
        self.assertEqual(r.returncode, 1)
        self.assertNotIn("CAN_CHECK_REACHED", r.stdout)

    def test_dry_ignores_issue(self):  # dry は捨て名で録る。一覧に無いコードを渡されても止めない
        r = self.collect("dry", ISSUE="T9")
        self.assertNotIn("[task]", r.stdout + r.stderr)
        self.assertIn("CAN_CHECK_REACHED", r.stdout)

    def test_no_registry_needs_no_issue(self):
        self.write("config/bimanual-collect.bottle.env", ENV.replace("TASK_SLUG=wood", "TASK_SLUG=bottle"))
        r = self.collect("session", "s2", env_file="config/bimanual-collect.bottle.env")
        self.assertNotIn("[task]", r.stdout + r.stderr)
        self.assertIn("CAN_CHECK_REACHED", r.stdout)

    def test_check_only(self):  # 録らずに突き合わせと名前の表示だけ（CAN に触らない）
        r = self.collect("session", "t1-l2-s1", CHECK_ONLY="true")
        self.assertEqual(r.returncode, 0, r.stderr)
        self.assertIn(f"[task] wood_t1-l2 → {BASE}", r.stdout)
        self.assertIn("CHECK_ONLY: mode=session task=wood issue=T1-L2", r.stdout)
        self.assertNotIn("CAN_CHECK_REACHED", r.stdout)

    def test_check_only_still_stops(self):
        r = self.collect("gate", CHECK_ONLY="true")
        self.assertEqual(r.returncode, 1)
        self.assertIn("課題コードが要る", r.stderr)


FAKE_RECORD = """#!/usr/bin/env bash
# lerobot-record の偽物: 受けた引数を 1 行ずつ出して終わる
printf 'FAKE_ARG %s\\n' "$@"
"""


class TestRecordName(Fixture):
    """record_bimanual_openarm.sh を一時の repo で最後まで回す（lerobot-record は偽物）。"""

    def setUp(self):
        super().setUp()
        for f in ("record_bimanual_openarm.sh", "task.py", "pose_set.py", "collect_manifest.py",
                  "record_console_filter.py", "lib/pose_set.sh"):
            self.write(f"scripts/{f}", (REPO / "scripts" / f).read_text(encoding="utf-8"))
        fake = self.write("bin/lerobot-record", FAKE_RECORD)
        fake.chmod(0o755)
        self.collections = self.tmp / "collections"

    def record(self, run: str = "r1", **kw) -> tuple[subprocess.CompletedProcess, Path]:
        run_dir = self.collections / run
        e = self.env(TASK="Pick up the wood and place it in the box", TASK_SLUG="wood", POSE_SET="new-set",
                     SCENE="desk-v1", OPERATOR="Kota", WRIST_RIGHT="true", WRIST_LEFT="true", WRIST_LEFT_SERIAL="111",
                     RUN_DIR=str(run_dir), LEROBOT_RECORD=str(self.repo / "bin/lerobot-record"),
                     PYTHON_BIN=sys.executable, SKIP_SYNC_CHECK="true", DMLOG="off", DISPLAY_DATA="false",
                     HOME=str(self.tmp), HF_LEROBOT_HOME=str(self.tmp / "hf"), CONSOLE_FILTER="off")
        for k in ("TASK_REPO", "REPO_ID", "RESUME", "DATASET_ROOT", "ALLOW_REPO_ID_MISMATCH"):
            e.pop(k, None)
        e.update(kw)
        r = subprocess.run(["bash", str(self.repo / "scripts/record_bimanual_openarm.sh"), "bi-wood-v1-s1"],
                           capture_output=True, text=True, env=e)
        return r, run_dir

    def repo_id_arg(self, run_dir: Path) -> str:
        log = (run_dir / "console.log").read_text(encoding="utf-8")
        return next(line.split("=", 1)[1] for line in log.splitlines() if line.startswith("FAKE_ARG --dataset.repo_id="))

    def manifest(self, run_dir: Path) -> dict:
        return json.loads((run_dir / "manifest.json").read_text(encoding="utf-8"))

    def test_registry_with_issue(self):
        r, run_dir = self.record(ISSUE="t1-l2")
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
        self.assertEqual(self.repo_id_arg(run_dir), BASE)  # lerobot の stamp_repo_id が末尾に _<日時> を足す
        m = self.manifest(run_dir)
        self.assertEqual(m["task_ledger"], {"task": "wood", "issue": "T1-L2", "external_id": "wood_t1-l2",
                                            "condition_version": "v2", "ledger": "config/tasks/wood.yaml"})
        self.assertEqual(m["artifacts"]["repo_id_requested"], BASE)
        self.assertEqual(m["people"]["operator"], "Kota")
        self.assertIn("ISSUE=T1-L2", (run_dir / "command.sh").read_text(encoding="utf-8"))
        self.assertEqual(run_dir.name, "r1")  # run_dir の名前は呼び出し側のまま（collect は <UTC>-<tag>）

    def test_no_registry(self):  # 台帳の無いタスクは今のまま（ISSUE を渡されても使わない）
        r, run_dir = self.record(TASK_SLUG="bottle", ISSUE="T1-L2")
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
        self.assertEqual(self.repo_id_arg(run_dir), "Orboh/bi_openarm_bottle_chest_wristr_wristl_v1")
        self.assertIsNone(self.manifest(run_dir)["task_ledger"])
        self.assertIn("台帳が無い", r.stderr)

    def test_registry_without_issue_warns(self):  # 直に呼んで ISSUE が無い → 警告して従来の名前
        r, run_dir = self.record()
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
        self.assertEqual(self.repo_id_arg(run_dir), "Orboh/bi_openarm_wood_chest_wristr_wristl_v1")
        self.assertIsNone(self.manifest(run_dir)["task_ledger"])
        self.assertIn("ISSUE（課題コード）が無い", r.stderr)

    def test_resume_keeps_stamped_name(self):
        stamped = f"{BASE}_20261028_101010"
        root = self.tmp / "hf" / stamped
        r, run_dir = self.record(ISSUE="T1-L2", RESUME="true", REPO_ID=stamped, DATASET_ROOT=str(root))
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
        self.assertEqual(self.repo_id_arg(run_dir), stamped)  # 足し直さない（足すと新しいデータセットになる）
        log = (run_dir / "console.log").read_text(encoding="utf-8")
        self.assertIn("FAKE_ARG --resume=true", log)
        self.assertIn(f"FAKE_ARG --dataset.root={root}", log)
        self.assertEqual(self.manifest(run_dir)["task_ledger"]["issue"], "T1-L2")
        self.assertIn(f"REPO_ID={stamped}", (run_dir / "command.sh").read_text(encoding="utf-8"))

    def test_resume_other_issue_stops(self):
        for i, other in enumerate(("Orboh/bi_openarm_wood_t2_chest_wristr_wristl_v2_20261028_101010",   # 別の課題コード
                                   "Orboh/bi_openarm_wood_t1-l2_chest_wristr_wristl_v1_20261027_090000",  # 旧い条件の版
                                   "Orboh/bi_openarm_wood_chest_wristr_wristl_v1_20261027_090000")):     # 課題コードの無い旧い名前
            with self.subTest(other=other):
                r, run_dir = self.record(run=f"mismatch{i}", ISSUE="T1-L2", RESUME="true", REPO_ID=other,
                                         DATASET_ROOT=str(self.tmp / "hf" / other))
                self.assertEqual(r.returncode, 1)
                self.assertIn("ALLOW_REPO_ID_MISMATCH", r.stderr)
                self.assertFalse(run_dir.exists())  # run_dir を作る前に止まる

    def test_resume_mismatch_allowed(self):
        other = "Orboh/bi_openarm_wood_chest_wristr_wristl_v1_20261027_090000"
        r, run_dir = self.record(ISSUE="T1-L2", RESUME="true", REPO_ID=other, DATASET_ROOT=str(self.tmp / "hf" / other),
                                 ALLOW_REPO_ID_MISMATCH="true")
        self.assertEqual(r.returncode, 0, r.stderr + r.stdout)
        self.assertEqual(self.repo_id_arg(run_dir), other)
        self.assertIn("警告", r.stderr)

    def test_camera_mismatch_stops(self):  # 台帳の cameras（chest_wristr_wristl）と今のカメラが違う
        r, run_dir = self.record(ISSUE="T1-L2", WRIST_LEFT="false")
        self.assertEqual(r.returncode, 1)
        self.assertIn("カメラ", r.stderr)
        self.assertFalse(run_dir.exists())

    def test_issue_not_listed_stops(self):
        r, run_dir = self.record(ISSUE="T9")
        self.assertEqual(r.returncode, 1)
        self.assertIn("T9 は wood の課題コードに無い", r.stderr)
        self.assertFalse(run_dir.exists())


class TestRealRegistryName(unittest.TestCase):
    def test_takeei_wood_fields(self):
        e = {k: v for k, v in os.environ.items() if k not in ("TASK_REPO", "PS_ROOT")}
        r = subprocess.run([sys.executable, str(REPO / "scripts/task.py"), "name", "takeei_wood", "t1-l2",
                            "--fields", "task,issue,id,version,dataset"], capture_output=True, text=True, env=e)
        self.assertEqual(r.stdout.split(), ["takeei_wood", "T1-L2", "takeei_wood_t1-l2", "v2",
                                            "Orboh/bi_openarm_takeei_wood_t1-l2_chest_wristr_wristl_v2"])


if __name__ == "__main__":
    unittest.main()
