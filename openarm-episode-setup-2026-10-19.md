# openarm-episode-setup-20261019 — 北九州・タケエイ仕分け汎化テスト エピソード収集 手順書

対象タスク: 机上に積まれた雑多な廃棄物(緩衝材・ブルーシート・円柱状の木材・発泡スチロール・ダンボール等、
約25品目)を1個ずつ拾い、アームの左右どちらか(厳密な置き先は決めない)へ移す汎化テスト。
開始姿勢・経路は板橋(takeei-v2)から引き継ぎ。目標 約200本(gate 10 → 50×3 → 40)。

このドキュメントは「ゼロから同じ状態を再現してデータ収集を再開する」ための手順書。上から順に実行すれば
2026-10-19時点の状態(バイラテラル・通知音・保存タイミング変更込み)で収集を始められる。

## 0. 前提: 2つのリポジトリにまたがる変更がある(重要)

今日の変更は **`~/projects/openarm-orboh`** と **`~/projects/lerobot`** の両方にまたがっている。
**どちらもまだコミット・push していない**。別のマシン/別の人がこの手順を再現する場合は、まずこの2つの
作業ツリーの差分(`git diff`)を適用するか、少なくとも以下を確認すること:

- `~/projects/openarm-orboh`: `main`ブランチ、`origin/main`より数コミット進行・未push。未コミットの変更あり
  (`scripts/collect_bimanual.sh`・`scripts/record_bimanual_openarm.sh`・`scripts/launch_bilateral.sh`(新規)・
  `scripts/build_teleop.sh`等のROS自動検出化・`config/bimanual-collect.kitakyushu_takeei_sort.env`)
- `~/projects/lerobot`: `feat/bi-openarm-soft-move-to`ブランチ、未コミットの変更あり
  (`src/lerobot/teleoperators/openarm_leader/`・`bi_openarm_leader/`・`src/lerobot/scripts/lerobot_record.py`・
  `src/lerobot/utils/utils.py`・`src/lerobot/common/control_utils.py`)

`git status` / `git diff` でこの2箇所を確認し、無ければ今日の変更(バイラテラル・通知音・保存タイミング)は
一切効かない(従来どおり重力補償のみ・無音・aの後保存、になる)。

## 1. 対象リソース

- scene: `scenes/kitakyushu-takeei-v1.yaml`
- pose_set: `config/pose_sets/kitakyushu-takeei-v2/`（status: draft、derived_from: takeei-v2。現在 CURRENT）
- env: `config/bimanual-collect.kitakyushu_takeei_sort.env`

## 2. 実機の起動手順(毎回)

1. 4本のアーム(leader/follower × 左右)の電源を入れる。非常停止ボタンが解除されていることを確認
   (特に左腕。過去にEボタンONのままハンドシェイク失敗した実績あり)
2. `cd ~/projects/openarm-orboh`
3. `export ENV_FILE=config/bimanual-collect.kitakyushu_takeei_sort.env`
   （新しいターミナル/タブでは消えるので、収集コマンドを打つ前に毎回確認する）
4. CAN健全性チェック: `sudo bash scripts/canup.sh`
   （`can_lead_r/l`・`can_fol_r/l`の4本がERROR-ACTIVE/WARNINGになっているか確認。異常なら実行を止める）
5. モータ応答チェック: `python scripts/openarm_motorcheck.py`
   （32モータが実値で応答しているか。未給電だと全0のまま`[OK]`と表示されることがあるので、数値そのものを
   目視する）
6. 昇降デスクの高さが900mmになっているか確認（当初740mmから、アームのリーチ確保のため変更済み）。
   アームを上げる動作をする際は机との干渉がないか手動で確認しながら進める
7. 山（対象物）の種類・個数が収集初日に確認した構成と大きく変わっていないか確認（厳密に同一である必要は
   ない。汎化テストのため）

ここまでで異常が無ければ、以降は`collect_bimanual.sh`が内部でカメラ露出調整・CAN再チェックを自動で行う。

## 3. 今日(2026-10-19)追加した機能と有効化方法

いずれも**既定はoff**（env変数を渡さなければ、10/18までと同じ重力補償のみ・無音・aの後保存の挙動になる）。
以下は実機確認済みの推奨設定。`collect_bimanual.sh`のコマンド実行時にまとめて環境変数として渡す。

```bash
PLAY_SOUNDS=true \
BILATERAL_FEEDBACK=true \
L_BILATERAL_GRIPPER_KP=1.25 L_BILATERAL_GRIPPER_KD=0.06 \
SAVE_ON_EPISODE_END=true \
GRIPPER_KP=12 \
bash scripts/collect_bimanual.sh gate 9   # 例: 続きのゲートを9本
```

- **`PLAY_SOUNDS=true`**（env ファイルに既定で設定済み・渡さなくても有効）: 操作イベントごとに通知音が鳴る。
  日本語音声合成(espeak-ng)の品質が低く聞き取れなかったため、言語非依存の通知音方式にした。
  起動時バナーに凡例が出る: `camera-shutter`=録画開始 / `message`=操作が必要(開始姿勢へ戻す・シーンを
  整える) / `complete`=完了(腕が戻った・録画終了) / `dialog-warning`=録り直し / `service-logout`=終了
- **`BILATERAL_FEEDBACK=true`**: followerの現在角へ向けた弱い位置引き込みをleaderに加える(力覚フィード
  バックの近似)。実機で段階調整済みの既定ゲイン(`~/projects/lerobot`の`config_openarm_leader.py`):
  腕 `bilateral_kp=[7.5,7.5,7.5,7.5,2.5,2.5,2.5]`、グリッパー `bilateral_gripper_kp=2.5`。
  **左右で感触差があったため、左グリッパーだけさらに弱める**(`L_BILATERAL_GRIPPER_KP=1.25
  L_BILATERAL_GRIPPER_KD=0.06`)。右はデフォルトのままでよい。
  注意: lerobot-recordの~30Hzループに相乗りさせた近似実装であり、本物のC++版(~500Hz)より緩い・
  追従遅れ自体が常時抵抗として乗る。ゲインを上げるとすぐ「重すぎる」感触になるので、変更する場合は
  小刻みに調整し振動が無いか確認すること
- **`SAVE_ON_EPISODE_END=true`**: エピソードの録画が終わった瞬間(右矢印 or 時間切れ)に即保存する。
  従来は「右矢印→aを押した後」に保存されていた。Ctrl-Cが効かないD-stateハング(既知の事象、本書§5参照)
  でデータを失わないための安全策。**トレードオフ: 左矢印での「直前のテイクを破棄」が使えなくなる**
  （間違えた場合は後から`lerobot-edit-dataset`で削除する。§6参照）。
  ⚠️ **lerobot側はユニットテストのみで、実機での収集セッションではまだ検証していない(2026-10-19時点)**。
  最初に使うときは`dry`モードで一度確認してから本番収集に使うこと

バイラテラルの手応えだけを単独で試したい場合（データは保存されない）は `scripts/launch_bilateral.sh` を
使う（README.md §D参照、本手順書とは別系統）。

## 4. 収集コマンド

```bash
# 配管・新機能の動作確認(2本・捨て名・SCENE不要)
PLAY_SOUNDS=true BILATERAL_FEEDBACK=true SAVE_ON_EPISODE_END=true \
  bash scripts/collect_bimanual.sh dry

# 受入ゲート10本（<slug>-v1-s1）→ 止めて検査
PLAY_SOUNDS=true BILATERAL_FEEDBACK=true L_BILATERAL_GRIPPER_KP=1.25 L_BILATERAL_GRIPPER_KD=0.06 \
  SAVE_ON_EPISODE_END=true GRIPPER_KP=12 \
  bash scripts/collect_bimanual.sh gate

# 続きのセッション
PLAY_SOUNDS=true BILATERAL_FEEDBACK=true L_BILATERAL_GRIPPER_KP=1.25 L_BILATERAL_GRIPPER_KD=0.06 \
  SAVE_ON_EPISODE_END=true GRIPPER_KP=12 \
  bash scripts/collect_bimanual.sh session s2 [本数]
```

- セッション配分(予定): gate 10本 → 50本×3セッション → 40本（合計200本。配分は現場で調整可）
- `GRIPPER_KP=12`: follower gripperの位置kp。既定25のまま硬い物を握り込むとコイルが過温ラッチする
  （板橋2026-09-21実測99℃）。硬い対象物があるため今回も12を使う
- キー操作: 右矢印=次のエピソード開始/終了 / `a`=両腕を開始姿勢へ復帰 / `esc`=収集終了
  （`SAVE_ON_EPISODE_END=true`の場合、左矢印での破棄は効かない。falseなら従来どおり左矢印で直前の
  テイクを破棄できる）
- 監視(任意・並走可): `python scripts/damiao_feedback_log.py can_lead_r can_fol_r can_lead_l can_fol_l --out <path>`

収集後、軽量メタをVaultへ回収: `bash scripts/pull_run.sh <UTC>-bi-kitakyushu-takeei-sort-v1-<sN>`

## 5. Ctrl-Cが効かない場合の対応（このセッション中に複数回発生した既知の事象）

USB-CANカーネルドライバがD-state（割り込み不可のスリープ）に入り、SIGINT/SIGTERM/SIGKILLいずれも
効かなくなることがある。`udev/81-openarm-usb-nosuspend.rules`を導入済みで緩和はされるが、根本解決
ではないため再発に備える。`SAVE_ON_EPISODE_END=true`にしておくと、ハング発生時点までに録り終えた
エピソードは既に保存済みなので失われない。

1. 本当に無反応か確認する（数秒待っても画面・CANに変化がないこと）
2. 4本のアーム（leader/follower × 左右）すべての電源を物理的に切る
3. 停止しないプロセスを掃除: `pkill -9 -f "lerobot-teleoperate|capture_start_pose_bimanual|openarm_pose_monitor|dmlog"`
4. 10秒待ってから電源を入れ直す
5. `sudo bash scripts/canup.sh`で健全性を再確認してから、§7の手順で収集を再開する

## 6. 誤って保存したエピソードの削除

`lerobot-edit-dataset --operation.type delete_episodes`は**全件削除を拒否する**（`ValueError: Cannot
delete all episodes from dataset`）。1本しか無い等で全消ししたい場合は、データセットディレクトリごと
削除する:

```bash
rm -rf ~/.cache/huggingface/lerobot/Orboh/<dataset_name>
```

## 7. 中断したゲート/セッションの再開（既存データセットへの追記）

`collect_bimanual.sh`は何も指定しないと**毎回タイムスタンプ付きの新規データセット**を作る（同じ`gate`を
再実行しても前回のデータには追記されない）。中断後に同じデータセットへ続きを録りたい場合は`RESUME=true`
+ 前回の実際の`REPO_ID`（タイムスタンプ込みの実名）+ `DATASET_ROOT`（実ディレクトリ）を明示する。
`NUM_EPISODES`（gateなら2番目の引数）には**今回追加で録る本数**を渡す（合計本数ではない）。

実例（2026-10-19、ゲート10本中1本撮れた状態から残り9本を再開したときの実際のコマンド）:

```bash
export ENV_FILE=config/bimanual-collect.kitakyushu_takeei_sort.env
RESUME=true \
REPO_ID=Orboh/bi_openarm_kitakyushu_takeei_sort_chest_wristr_wristl_v1_20261019_160612 \
DATASET_ROOT=$HOME/.cache/huggingface/lerobot/Orboh/bi_openarm_kitakyushu_takeei_sort_chest_wristr_wristl_v1_20261019_160612 \
GRIPPER_KP=12 \
bash scripts/collect_bimanual.sh gate 9
```

実際の`REPO_ID`/`DATASET_ROOT`は`ls ~/.cache/huggingface/lerobot/Orboh/`で該当データセットのディレクトリ名
（`bi_openarm_<task_slug>_<obs>_v1_<YYYYMMDD_HHMMSS>`）を確認して差し替える。§3の新機能のenv変数も
同時に渡してよい。

## Git状態（2026-10-19時点）

- `~/projects/openarm-orboh`: `main`ブランチ（別ブランチは切っていない）。origin/mainより数コミット進行・
  未push。本書§0に挙げたファイルが未コミット
- `~/projects/lerobot`: `feat/bi-openarm-soft-move-to`ブランチ。本書§0に挙げたファイルが未コミット
- pushのタイミングはチームで相談してから（2026-10-19時点では未実施）

## 関連

- README.md §D「バイラテラル(力覚フィードバック)の手応え確認」— `launch_bilateral.sh`の使い方
- Issue #19 — ROS distro決め打ち(`/opt/ros/jazzy`)の自動検出化修正、`launch_bilateral.sh`新規作成
- `scenes/kitakyushu-takeei-v1.yaml` — シーン詳細（机・カメラ・対象物・配分）
- `config/pose_sets/kitakyushu-takeei-v2/` — 開始姿勢・経路（takeei-v2から引き継ぎ）
- Dev Log: Obsidian `toyota-body-orboh` vault `Free-space/ueda/2026-10-19.md`（今日の作業の経緯・試行錯誤の詳細）
