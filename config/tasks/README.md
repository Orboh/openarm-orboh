# タスクの台帳（config/tasks/）

1 タスク = 1 ファイル（`<タスク ID>.yaml`）。タスク ID・指示文・タスクファイル（`config/bimanual-collect.<slug>.env`）・カメラ・このタスクの課題コード（録る・録らないの区別は課題リストと台帳の状態で持つ）・条件の版（開始姿勢セットとシーンの組）・名前の型を 1 か所に置く。

タスクファイル・開始姿勢セット・シーン YAML はそれぞれ別の場所にあり、以前は「今どの組で録るか」を人が覚えていた。組を取り違えると、別の条件のデータが同じ名前のデータセットに混ざる。ここに組を書き、`scripts/task.py check` が収集の前に突き合わせる。

- 命名則の正本は orboh-company-vault。共有の写しは `Orboh/doma-sorting-dev` の `07-collection/collection-naming-rules.md`「命名則」（2026-10-28・2026-10-31 に移した）。課題コードの定義は各タスクの `issue_list` の文書（課題リスト）、どのエピソードをどのデータセットに入れたかは `ledger` の文書（台帳・2026-10-31 に課題リストから分けた）。`ledger` が無いタスクは `issue_list` の文書を台帳として見る
- 書式: `config/pose_sets/*/set.yaml` と同じ「1 行 1 キー・入れ子なし」（PyYAML 不要）。一覧はカンマ区切り、条件の版は `v<N>.pose_set` / `v<N>.scene` / `v<N>.note`

---

## 名前

| もの | 型 | 例 |
|---|---|---|
| 外に見せる ID | `<タスク ID>_<課題コードを小文字>` | `takeei_wood_t1-l2` |
| HF のデータセット | `Orboh/bi_openarm_<外に見せる ID>_<カメラ>_v<条件の版>` | `Orboh/bi_openarm_takeei_wood_t1-l2_chest_wristr_wristl_v2` |
| HF のモデル | `Orboh/gr00t-bi-openarm-<外に見せる ID をハイフンに>-v<N>` | `Orboh/gr00t-bi-openarm-takeei-wood-t2-v1` |
| 複数の課題コードを混ぜたモデル | `Orboh/gr00t-bi-openarm-<タスク>-mix-v<N>` | `Orboh/gr00t-bi-openarm-takeei-wood-mix-v1` |

---

## 使い方

| やること | コマンド |
|---|---|
| タスクと課題コードの一覧（viser などから読むなら `--json`） | `python3 scripts/task.py list [--json]` |
| 収集の前の突き合わせ（食い違えば理由を並べて終了コード 1） | `python3 scripts/task.py check takeei_wood [--issue T1-L2]` |
| 名前を出す | `python3 scripts/task.py name takeei_wood T1-L2 [--model-version 2] [--mix]` |
| 課題コードを決める（ISSUE かタグから） | `python3 scripts/task.py issue takeei_wood [--issue T1-L2] [--tag t1-l2-s1]` |

- `scripts/collect_bimanual.sh` は gate / session の前に `task.py check` を自動で呼ぶ（タスクファイルの `TASK_SLUG` に当たる `config/tasks/<TASK_SLUG>.yaml` があるときだけ。dry では呼ばない）
- 台帳のあるタスクの gate / session は**課題コードが必須**（2026-10-28）。`ISSUE=T1-L2` があればそれ、無ければ session のタグから読む（`t1-l2-s1` → `T1-L2`・`t2-s1` → `T2`・`t1-l0a-s1` → `T1-L0a`）。どちらからも分からない・一覧に無い・`ISSUE` とタグで食い違う → 止める（`python3 scripts/task.py issue <task> [--issue ..] [--tag ..]` が決める）。dry は課題コードを使わない
- 決まった課題コードで、データセットの名前は `Orboh/bi_openarm_<外に見せる ID>_<カメラ>_v<条件の版>_<日時>`（`_<日時>` は lerobot が足す）、run_dir の `manifest.json` には `task_ledger`（`task`・`issue`・`external_id`・`condition_version`・`ledger`）が入る。run_dir の名前（`<UTC>-bi-<slug>-<tag>`）は変わらない
- 途中から再開（`RESUME=true REPO_ID=<stamp 後の実名> DATASET_ROOT=...`）は `REPO_ID` をそのまま使う。今の課題コードと条件の版の名前で始まらない `REPO_ID` へは足さずに止める（承知なら `ALLOW_REPO_ID_MISMATCH=true`）
- 録らずに確かめる: `CHECK_ONLY=true ISSUE=T1-L2 bash scripts/collect_bimanual.sh gate`（突き合わせと名前の表示だけ。CAN・カメラに触らない）
- 課題コードは大文字・小文字を問わずに受け、台帳の書き方（`T1-L0a` など）にそろえる
- `--issue` を渡したときは、課題リストと台帳の文書が手元の clone（既定 `~/projects/doma-sorting-dev`・`TASK_LEDGER_DIR` で変えられる）にあれば、その課題コードの節があるかを見る。文書や節が無ければ警告だけで止めない
- 条件の版を上げたとき（開始姿勢セット・シーン・カメラのどれかを変えた）: `v<N+1>.pose_set` / `v<N+1>.scene` を足し、`current_version` を変え、タスクファイルの `POSE_SET` / `SCENE` も合わせる
