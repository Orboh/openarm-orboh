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
| `openarm_cameras.py` | カメラ構成の正本（env の解釈・データセットからの検出・modality の組み立て）。gr00t 非依存 |
| `openarm_config.py` | `NEW_EMBODIMENT` として OpenArm を登録する。学習コマンドの `--modality_config_path` に渡す。カメラは `OPENARM_CAMERAS` |
| `gen_modality_json.py` | 変換済みデータセットの `meta/info.json` からカメラを検出して `meta/modality.json` を書く（手コピーの置き換え） |
| `check_dataloader.py` | モデル抜きで dataloader が1ステップ読めるかの疎通確認（G1 の PASS 判定）。先にカメラ不一致を検出する |
| `modality.json` | chest 単眼のスナップショット（50ep v2 を作ったときのもの）。新規は `gen_modality_json.py` で作る |

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
