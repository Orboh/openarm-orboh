# gr00t/ — GR00T に OpenArm という身体を教えるための設定

`openarm_embodiment/` は GR00T（Isaac-GR00T）を OpenArm で回すのに必須の設定一式。
**これまで各マシンの Isaac-GR00T clone の中に未追跡で置かれており、どの repo にも入っていなかった**
（clone しても揃わず、既存マシンからコピーするしかない状態だった）。ここを正本にする。

## カメラ構成ごとに2系統ある

**chest 単眼と chest+wrist は入力次元が違う＝別モデルになる。** 混ぜて使うと dataloader が
モダリティキー不一致で落ちるか、学習済みモデルと噛み合わない。データセットを録ったときの
カメラ構成に合わせて選ぶ。

| 用途 | modality | config |
|---|---|---|
| **chest 単眼**（`openarm_bottle_pnp_chest_v2` = 50ep の本線モデル） | `modality.json` | `openarm_config.py` |
| **chest + wrist_right**（D405 手首カメラ併用・遠隔/PICO 収集の2カメラ構成） | `modality_wrist.json` | `openarm_config_wrist.py` |

疎通確認は共通で `check_dataloader.py`（モデル抜きで dataloader が1ステップ読めるかを見る。
G1 の PASS 判定＝設計書 §3.5）。

## 使い方

Isaac-GR00T の clone は `openarm_embodiment/` を**リポジトリ直下**に置く前提で動く
（`--modality_config_path openarm_embodiment/openarm_config.py`）。配置はスクリプトで行う:

```bash
# 既定の GR00T_ROOT は ~/projects/Isaac-GR00T-n1.7
bash scripts/install_gr00t_embodiment.sh
# 別の clone に入れるとき
GR00T_ROOT=~/projects/Isaac-GR00T bash scripts/install_gr00t_embodiment.sh
```

symlink を張るので、以後の更新はこのリポジトリ側を直せば全マシンに効く。

> **⚠ 既存マシンで配置スクリプトを走らせるときの注意**
> 2026-09-05 時点の Thor は `openarm_embodiment/openarm_config.py` が **wrist 版**（chest+wrist）に
> なっている。symlink に差し替えると、この名前は **chest 単眼版**を指すようになる。
> wrist 構成で回すときは、以後は明示的に `openarm_config_wrist.py` / `modality_wrist.json` を指すこと。
> （wrist 版の中身は Thor 現物と md5 一致で取り込んであるので、設定値そのものは失われない）

## 入れていないもの

EC2 の G2 runbook と autostop スクリプトは、インスタンス ID を含み陳腐化しやすいので
ここには置かない。正本は Vault 側の設計書 `gr00t-openarm-pipeline-design.md` §4.4・§4.5。
