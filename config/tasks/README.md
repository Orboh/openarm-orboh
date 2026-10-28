# タスクの台帳（config/tasks/）

1 タスク = 1 ファイル（`<タスク ID>.yaml`）。タスク ID・指示文・タスクファイル（`config/bimanual-collect.<slug>.env`）・カメラ・このタスクの課題コード（録る・録らないの区別は課題リストと台帳の状態で持つ）・条件の版（開始姿勢セットとシーンの組）・名前の型を 1 か所に置く。

タスクファイル・開始姿勢セット・シーン YAML はそれぞれ別の場所にあり、以前は「今どの組で録るか」を人が覚えていた。組を取り違えると、別の条件のデータが同じ名前のデータセットに混ざる。ここに組を書き、`scripts/task.py check` が収集の前に突き合わせる。

- 命名則の正本: 共有 repo `Orboh/doma-sorting-dev` の `07-collection/README.md`「命名則」（2026-10-28）。課題コードの定義と、どのエピソードをどのデータセットに入れたかの台帳は、各タスクの `issue_list` の文書
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

- `scripts/collect_bimanual.sh` は gate / session の前に `task.py check` を自動で呼ぶ（タスクファイルの `TASK_SLUG` に当たる `config/tasks/<TASK_SLUG>.yaml` があるときだけ。dry では呼ばない）。`ISSUE=T1-L2` を渡すと、その課題コードがこのタスクの一覧にあるかも見る
- 課題コードは大文字・小文字を問わずに受け、台帳の書き方（`T1-L0a` など）にそろえる
- `--issue` を渡したときは、課題リストと台帳の文書が手元の clone（既定 `~/projects/doma-sorting-dev`・`TASK_LEDGER_DIR` で変えられる）にあれば、その課題コードの節があるかを見る。文書や節が無ければ警告だけで止めない
- 条件の版を上げたとき（開始姿勢セット・シーン・カメラのどれかを変えた）: `v<N+1>.pose_set` / `v<N+1>.scene` を足し、`current_version` を変え、タスクファイルの `POSE_SET` / `SCENE` も合わせる
