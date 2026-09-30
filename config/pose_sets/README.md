# 開始姿勢セット（config/pose_sets/）

1 モデル = 1 セット。学習データを録った時の「開始姿勢・較正・コード・モデル・データセット」の組を 1 フォルダに置く。

モデルは、収集時と同じ開始姿勢と同じ較正の上でしか正しく動かない（同じ関節角の数値が違う物理姿勢を指すと、学習曲線にもエラーにも出ないまま系統的にずれる）。以前は全スクリプトが `config/initial_pose_{left,right}.yaml` という 1 組の固定名を読み、撮り直すたびに中身を上書きしていたため、過去のモデルを回すと黙って新しい姿勢で起動していた。2026-09-30 にセットへ移した。

## 中身

| ファイル | 内容 |
|---|---|
| `left.yaml` / `right.yaml` | 開始姿勢（度・7 関節＋gripper・`waypoints:` で立ち上がりの経由地を持てる）。**学習済みモデルと組になったら書き換えない** |
| `set.yaml` | モデル・データセット・収集ディレクトリ・コードの commit・較正の sha（先頭 12 桁）・指示文（`lang`）・注記。1 行 1 キー |
| `calib/` | 収集時の較正ファイルの写し。較正を作り直した後にそのモデルを回すときに戻す |
| `CURRENT`（フォルダ直下） | テレオペ・収集が `POSE_SET` 未指定のときに使うセット名 |

## 使い方

- 推論（`scripts/g4/run_rollout.sh`・`scripts/run_rollout_lerobot.sh`）: `POSE_SET=<name>` 必須。モデル・開始姿勢はセットから決まり、較正の sha が合わなければ止まる（承知の上なら `ALLOW_CALIB_MISMATCH=true`）。モデルを別に指定してセットと違えば止まる（`ALLOW_MODEL_MISMATCH=true`）。セットに `lang` があれば `--lang_instruction` と一字一句の一致を見る
- テレオペ・収集: `POSE_SET` 未指定なら `CURRENT`。収集は較正の一致を必須にする
- 新しい姿勢を撮る: 新しいセット名で撮る（`POSE_SET=<新名> bash scripts/capture_initial_pose.sh <side>`）。既存ファイルは上書きしない
- 解決の実装: `scripts/lib/pose_set.sh`

## セット一覧（2026-09-30）

| セット | モデル | 較正 |
|---|---|---|
| bottle-v2 | gr00t-bottle-v2 | 右 follower の収集時較正が現存しない（今の較正では収集時の座標で再生できない） |
| tape-v1 | gr00t-tape-pnp-v1 | 同上 |
| bi-blockstack-v1 | gr00t-bi-blockstack-v1 | 一致（2026-09-21〜22 bump-to-stop） |
| bi-blockstack-leftarm-v1 | gr00t-bi-blockstack-leftarm-v1 | 一致（同上）。右腕はカメラ役で経由地つき |
