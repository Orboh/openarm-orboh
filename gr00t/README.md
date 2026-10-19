# gr00t/ — GR00T に OpenArm という身体を教えるための設定

`openarm_embodiment/` は GR00T（Isaac-GR00T）を OpenArm で回すのに必須の設定一式。
**これまで各マシンの Isaac-GR00T clone の中に未追跡で置かれており、どの repo にも入っていなかった**
（clone しても揃わず、既存マシンからコピーするしかない状態だった）。ここを正本にする。

## カメラ構成は `OPENARM_CAMERAS` 1本で決める

カメラは今後 wrist が既定になり、3台・4台と増えていく。組み合わせごとにファイルを持つと破綻するので、
**構成はカンマ区切りの文字列1つで指定し、`modality.json` はデータセットから生成する**。

| 場面 | 指定 |
|---|---|
| **既定**（2026-09-05 以降の標準＝chest＋wrist_right） | 何も付けない |
| chest 単眼（`openarm_bottle_pnp_chest_v2`＝50ep 本線モデルの再現） | `OPENARM_CAMERAS=chest` |
| 3台以上に増やす | `OPENARM_CAMERAS=chest,wrist_right,overhead` のように名前を足す |

カメラ名は LeRobot の `observation.images.<name>` の `<name>`（＝録画時に record スクリプトへ渡したカメラキー）。
**構成が違えば入力次元が違う別モデル**になる。学習に使うデータセットと必ず一致させる（不一致は `check_dataloader.py` が止める）。

| ファイル | 役割 |
|---|---|
| `openarm_layout.py` | 状態/行動レイアウト（単腕8DoF/両腕16DoF）の正本（env の解釈・データセットからの自動判定・groups の組み立て）。gr00t 非依存 |
| `openarm_cameras.py` | カメラ構成の正本（env の解釈・データセットからの検出・modality の組み立て）。gr00t 非依存 |
| `openarm_config.py` | `NEW_EMBODIMENT` として OpenArm を登録する。学習コマンドの `--modality_config_path` に渡す。カメラは `OPENARM_CAMERAS`、レイアウトは `OPENARM_EMBODIMENT` |
| `gen_modality_json.py` | 変換済みデータセットの `meta/info.json` からカメラ・レイアウトを検出して `meta/modality.json` を書く（手コピーの置き換え） |
| `check_dataloader.py` | モデル抜きで dataloader が1ステップ読めるかの疎通確認（G1 の PASS 判定）。先にカメラ・embodiment 不一致を検出する |
| `modality.json` | chest 単眼のスナップショット（50ep v2 を作ったときのもの）。新規は `gen_modality_json.py` で作る |

## 両腕（bimanual）データを回す

単腕（従来）は `joint_1..7.pos` + `gripper.pos` の8次元。両腕は `OPENARM_EMBODIMENT` で切り替える。

| `OPENARM_EMBODIMENT` | 次元 | state/action names |
|---|---|---|
| 未指定（既定） | 8（単腕） | `joint_1.pos`〜`joint_7.pos`, `gripper.pos` |
| `bimanual` | 16（両腕） | `right_joint_1.pos`〜`right_joint_7.pos`, `right_gripper.pos`, `left_joint_1.pos`〜`left_joint_7.pos`, `left_gripper.pos` |

16次元の契約（index / lerobot feature name / modality key）:

| index | lerobot feature name | modality key |
|---|---|---|
| 0–6 | `right_joint_1.pos`〜`right_joint_7.pos` | `right_arm`（0:7） |
| 7 | `right_gripper.pos` | `right_gripper`（7:8） |
| 8–14 | `left_joint_1.pos`〜`left_joint_7.pos` | `left_arm`（8:15） |
| 15 | `left_gripper.pos` | `left_gripper`（15:16） |

この並び（right 先・left 後）の正本は lerobot 側 `BiOpenArmFollower._motors_ft`（bi_openarm_follower.py）と
`lerobot_record.py`（dataset_features を robot.action_features / observation_features から組む）。
teleop（`BiOpenArmLeader`）側は left 先の並びだが、dataset 側の並びには影響しない。

**生成手順**（layout は `meta/info.json` の `observation.state` names から自動判定される。
env を付けると判定結果との一致を検証するだけ）:

```bash
python openarm_embodiment/gen_modality_json.py ~/.cache/huggingface/lerobot/Orboh/<bimanual_dataset>
```

**疎通確認**:

```bash
OPENARM_EMBODIMENT=bimanual uv run --no-sync python openarm_embodiment/check_dataloader.py ~/.cache/huggingface/lerobot/Orboh/<bimanual_dataset>
```

**学習**（カメラは既定の `chest,wrist_right` のまま。先頭に env を付けるだけ）:

```bash
OPENARM_EMBODIMENT=bimanual <学習コマンド> --modality_config_path openarm_embodiment/openarm_config.py
```

**推論**は `scripts/g4/eval_openarm.py` の `--robot.type=bi_openarm_follower` 対応（別途）を参照。

落とし穴:
- `OPENARM_EMBODIMENT` の付け忘れ（単腕設定のまま16次元データを回す等）は `gen_modality_json.py` /
  `check_dataloader.py` がどちらも学習前に `[FAIL]` で止める。
- v3→v2 変換（Isaac-GR00T `scripts/lerobot_conversion/convert_v3_to_v2.py`）は `observation.state` の
  次元や names に触らず、video も `ffmpeg -c copy` でそのまま切り出す（AV1 のまま）。16次元・複数
  state キーだからといって変換スクリプト側の追加対応は要らない。

## 使い方

### 1. GR00T clone への配置（マシンごとに1回）

Isaac-GR00T の clone は `openarm_embodiment/` を**リポジトリ直下**に置く前提で動く。symlink で配置する:

```bash
# 既定の GR00T_ROOT は ~/projects/Isaac-GR00T-n1.7
bash scripts/install_gr00t_embodiment.sh
# 別の clone に入れるとき
GR00T_ROOT=~/projects/Isaac-GR00T bash scripts/install_gr00t_embodiment.sh
```

以後の更新はこのリポジトリ側を直せば全マシンに効く。

### 2. データセットごとに `modality.json` を生成する（v3→v2 変換の直後）

```bash
cd <GR00T_ROOT>
python openarm_embodiment/gen_modality_json.py ~/.cache/huggingface/lerobot/Orboh/<dataset>
# データセットにある全カメラを使う。一部だけ使うなら OPENARM_CAMERAS=chest を付ける
```

### 3. 疎通確認（学習前に1回）

```bash
uv run --no-sync python openarm_embodiment/check_dataloader.py ~/.cache/huggingface/lerobot/Orboh/<dataset>
# chest 単眼データなら OPENARM_CAMERAS=chest を付ける。構成が食い違えば [FAIL] で止まる
```

### 4. 学習・推論

学習コマンドに `--modality_config_path openarm_embodiment/openarm_config.py` を渡す。
データセットが chest 単眼のときだけ **同じコマンドの先頭に** `OPENARM_CAMERAS=chest` を付ける。
それ以外（既定構成）は何も付けない。

> **⚠ 既存マシンで配置スクリプトを走らせるときの注意（2026-09-05）**
> Thor の `openarm_embodiment/` は実体ディレクトリで、`openarm_config.py` は chest＋wrist_right を直書きした版。
> symlink に差し替えても**既定が chest＋wrist_right なので挙動は変わらない**。
> 変わるのは chest 単眼を回すときで、以後は `OPENARM_CAMERAS=chest` を明示する。

## 入れていないもの

EC2 の G2 runbook と autostop スクリプトは、インスタンス ID を含み陳腐化しやすいので
ここには置かない。正本は Vault 側の設計書 `gr00t-openarm-pipeline-design.md` §4.4・§4.5。
