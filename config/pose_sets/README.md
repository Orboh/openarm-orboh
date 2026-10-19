# 開始姿勢セット（config/pose_sets/）

1 モデル = 1 セット。学習データを録った時の「開始姿勢・較正・シーン（環境 YAML）・コードの版・データセット・モデル」の組を 1 フォルダに置く。

モデルは、収集時と同じ開始姿勢・同じ較正・同じシーンの上でしか正しく動かない（較正が違うと同じ関節角の数値が違う物理姿勢を指し、学習曲線にもエラーにも出ないまま系統的にずれる）。以前は全スクリプトが `config/initial_pose_{left,right}.yaml` という 1 組の固定名を読み、撮り直すたびに中身を上書きしていた。較正ファイルも同じ名前へ上書きされ、2026-09-21 には右腕の 08-04 版が退避なしで消えた。2026-09-30 にセットへ移した。

---

## 中身

| ファイル | 内容 |
|---|---|
| `left.yaml` / `right.yaml` | 開始姿勢（度・7 関節＋gripper）。`waypoints:` で立ち上がりの経由地を持てる |
| `calib/*.json` | 較正ファイルの写し（follower / leader × left / right） |
| `scene.yaml` | シーンプロファイル `scenes/<scene>.yaml` の写し |
| `set.yaml` | 状態・上の sha・コードの版・データセット・モデル・指示文（`lang`）・注記。1 行 1 キー |
| `CURRENT`（フォルダ直下） | テレオペ・収集が `POSE_SET` 未指定のときに使うセット名 |

---

## 状態: draft と frozen

- **draft**: まだ本番の収集をしていない。収集を始めるたびに、今の較正・シーン・コードの版を自動で取り込み直す
- **frozen**: 本番の収集（SCENE が none でなく、データセットが作られた run）を 1 回でも行うと、record が自動で凍結する。以後、開始姿勢・較正・シーンが 1 つでも凍結時と違えば、収集も推論も止まる
- **条件を変えたとき**（較正をやり直した・シーン YAML を変えた・姿勢を撮り直した）: 凍結したセットはそのまま残し、新しいセットを作る。`calibrate_openarm.sh` は上書き前に較正ファイルを `.bak-<日時>` へ退避し、終わりに新しいセットの作り方を表示する

---

## 使い方

| やること | コマンド |
|---|---|
| 一覧（`*` が CURRENT） | `python3 scripts/pose_set.py list` |
| **両腕の開始姿勢と立ち上がり経路をテレオペ 1 回で撮る（推奨）** | `bash scripts/capture_start_pose_bimanual.sh <新名> [シーン]`（出発姿勢 capture-base へ整列、整列完了から自動で記録、Ctrl-C か esc で終えると抽出と警告、y/N で書き出しと CURRENT の切り替え・2026-10-02） |
| 中身と sha | `python3 scripts/pose_set.py show <set>` |
| 記録済みの CSV から作り直す（撮影スクリプトの中身と同じ） | `python3 scripts/pose_from_dwell.py <csv> --set <新名> --scene <scene> --write` |
| 既存セットの姿勢を引き継いで作る（較正・シーンを変えた後） | `python3 scripts/pose_set.py new <新名> --from <set>` |
| 片腕ずつ手で撮る | `POSE_SET=<新名> bash scripts/capture_initial_pose.sh <side>` |
| 使うセットを切り替える | `python3 scripts/pose_set.py current <set>` |
| 学習後にモデルを登録 | `python3 scripts/pose_set.py set-model <set> --model-dir ~/models/<name> --model-hf <repo> --lang "<指示文>"` |
| 前のセットのモデルを回すため較正を戻す | `python3 scripts/pose_set.py restore-calib <set>`（今の較正は `.bak-<日時>` へ） |

- 推論（`scripts/g4/run_rollout.sh`・`scripts/run_rollout_lerobot.sh`）: `POSE_SET=<set>` 必須。モデル・開始姿勢はセットから決まり、較正が違えば止まる（承知の上なら `ALLOW_CALIB_MISMATCH=true`）。別のモデルを指定してセットと違えば止まる（`ALLOW_MODEL_MISMATCH=true`）。セットに `lang` があれば `--lang_instruction` と一字一句の一致を見る
- テレオペ: `POSE_SET` 未指定なら CURRENT。較正の不一致は警告だけ
- 解決の実装: `scripts/lib/pose_set.sh`（bash）と `scripts/pose_set.py`（照合・取り込み・凍結）

---

## セット一覧（2026-10-03）

| セット | 状態 | モデル | シーン | 較正 |
|---|---|---|---|---|
| bottle-v2 | frozen | gr00t-bottle-v2 | bottle-desk-v1 | follower_right は 08-04 dev-log の表から復元（原本なし・今との差 最大 1.79°）。leader_right は原本 |
| tape-v1 | frozen | gr00t-tape-pnp-v1 | tape-desk-v1 | 同上 |
| bi-blockstack-v1 | frozen | gr00t-bi-blockstack-v1 | itabashi-blockstack-v1 | 原本 4 本（2026-09-21〜22 bump-to-stop） |
| bi-blockstack-leftarm-v1 | frozen | gr00t-bi-blockstack-leftarm-v1 | itabashi-blockstack-leftarm-rightcam-v1 | 同上。右腕はカメラ役で経由地つき |
| capture-base | frozen | なし（撮影の出発姿勢専用・収集と推論には使わない） | なし | 照合しない（テレオペは警告のみ） |
| takeei-v1 | draft（Thor の作業ツリーのみ・未 commit） | 未学習 | 未設定 | 2026-10-01 撮影・肘 J4 が可動域の端の 1° 手前で不採用 |
| takeei-v2 | frozen・CURRENT（2026-10-06 13:53） | gr00t-bi-takeei-sort-v1（16 step）・gr00t-bi-takeei-sort-h32-v1（32 step） | itabashi-takeei-v1 | 2026-10-02 撮影・作業面 108・経由地 左右 10 点。データセットは 50 本の `..._20261006_140857` |
