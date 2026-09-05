# gr00t/ — GR00T に OpenArm という身体を教えるための設定

`openarm_embodiment/` は GR00T（Isaac-GR00T）を OpenArm で回すのに必須の設定一式。
**これまで各マシンの Isaac-GR00T clone の中に未追跡で置かれており、どの repo にも入っていなかった**
（clone しても揃わず、既存マシンからコピーするしかない状態だった）。ここを正本にする。

| ファイル | 役割 |
|---|---|
| `openarm_embodiment/modality.json` | 8次元のどこが腕（0-6）でどこがグリッパ（7）か、カメラキー、指示文の在処の対応表 |
| `openarm_embodiment/openarm_config.py` | `NEW_EMBODIMENT` として OpenArm を登録する。single_arm=RELATIVE / gripper=ABSOLUTE |
| `openarm_embodiment/check_dataloader.py` | モデル抜きで dataloader が1ステップ読めるかの疎通確認（G1 の PASS 判定） |

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

## 入れていないもの

EC2 の G2 runbook と autostop スクリプトは、インスタンス ID を含み陳腐化しやすいので
ここには置かない。正本は Vault 側の設計書 `gr00t-openarm-pipeline-design.md` §4.4・§4.5。
