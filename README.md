# openarm-orboh

OpenArm（enactic）のテレオペ・収集を Orboh 運用向けにまとめた**入口リポジトリ**。単体では完結せず、2本の兄弟リポと組み合わせて使う:

| リポジトリ | ブランチ | 役割 |
|---|---|---|
| **openarm-orboh**（このリポ） | `main` | submodule（`openarm_teleop`/`openarm_description`）・CAN 固定名運用・起動スクリプト一式・シーンプロファイル・IK |
| [Orboh/lerobot](https://github.com/Orboh/lerobot) | `feat/openarm-remote-teleop` | `openarm_follower`/`openarm_leader`/`openarm_leader_remote` の実装・重力補償・`lerobot-record` 本体。**upstream の huggingface/lerobot ではなくこの fork 限定**（headless キー入力・遠隔テレオペが upstream には無い） |
| [Orboh/XRoboToolkit-Orin-Video-Sender](https://github.com/Orboh/XRoboToolkit-Orin-Video-Sender) | `vector-x86` | PC 接続カメラの映像を PICO へ H.264 配信する sender。**PICO 映像経路を使うマシン（主に Thor）にのみ必要** |

| マシン | 役割 | 必要なリポジトリ |
|---|---|---|
| Thor（follower 実機側） | フォロワー駆動・カメラ・収集 | 上記3本すべて（sender は PICO 映像を使う場合のみ） |
| 操作側 PC（leader 側） | リーダー駆動・重力補償 | openarm-orboh + lerobot の2本 |

## クイックスタート（1回の clone から）

```bash
mkdir -p ~/projects && cd ~/projects
git clone --recurse-submodules https://github.com/Orboh/openarm-orboh.git
cd openarm-orboh
bash scripts/bootstrap.sh              # Thor（follower実機側）相当: 3リポ+lerobot venv+senderビルド+setup.sh
```

操作側 PC（leader 側。sender は不要）:

```bash
bash scripts/bootstrap.sh --operator
```

`scripts/bootstrap.sh` がやること: 兄弟リポ（lerobot / XRoboToolkit-Orin-Video-Sender、上表のブランチに pin 済み）の clone、lerobot venv 構築（`uv` があれば優先、無ければ `python3 -m venv` フォールバック。`damiao`+`intelrealsense` extra、pinocchio、pyzmq を入れる）、sender のビルド（gst 系依存が揃っていれば）、`scripts/setup.sh`（apt 依存・`openarm_teleop` ビルド・udev ルール導入）までを1本で通し、最後に OK/WARN の検証サマリと「次にやること」チェックリストを表示する。

- **冪等・喪失ゼロ**: 既にある兄弟リポは fetch と状態報告のみ（ahead/behind/dirty）。checkout / reset / pull / 削除は一切しない。dirty なリポは警告して触らない。
- `--check`: 読み取り専用の状態レポートのみ（fetch もしない・一切書き込まない）
- `--skip-setup`: `scripts/setup.sh`（sudo・apt を含む）を飛ばす
- `--shallow`: 兄弟リポを `--depth 1` で clone する（**fresh-clone の動作確認専用。本番機では使わない**）
- `PROJECTS_DIR` 環境変数: リポの基底を上書き（既定 `$HOME/projects`。**テスト用途専用**。実行スクリプト群は `$HOME/projects` を決め打ちしている）

### bootstrap.sh がやらないこと（次にやること）

ハードウェア・アカウント・現地環境に関わる手順は自動化していない。`bootstrap.sh` の末尾にも同じチェックリストが出る。

1. PEAK アダプタを PC に直挿し（USB ハブ経由は NG）。新品（役割タグ未設定）のみ焼く: `sudo bash scripts/burn_channel_id.sh <iface> leader|follower`
2. CAN を固定名にして健全性を確認: `sudo bash scripts/canup.sh`
3. 較正: 既存機がある場合は `leader_*.json` を scp でコピー、無い（新規機体の）場合は `bash scripts/calibrate_openarm.sh <left|right>`
4. Hugging Face: 保存先リポジトリを Web UI で作成（Owner=Orboh）し、トークンにそのリポの write 権限を付与してから小さなテスト push で確認する（新規リポ名だと 403 になる罠がある）
5. PICO: 開発者モード有効化・Safeguard オフ・XRoboToolkit APK 導入、アプリで Remote Vision → Listen → Thor の IP:13579 → Confirm
6. ネットワーク: 操作側 PC と Thor を同一 5GHz Wi-Fi/LAN に接続し、必要なら `sudo ufw allow 13579/tcp`
7. カメラ製造番号（D435i/D405 のシリアル）がシーンプロファイル（`scenes/*.yaml`）の値と一致するか目視確認
8. 安全ゲート: 可動域クリア・非常停止（電源スイッチ）への到達確認・グリッパーに異物が挟まっていないことの確認

## 機能別の最短経路

### A. PICO + 遠隔テレオペ

```bash
# Thor: 実カメラ2台を仮想カメラ3本へ分配（PICO用PiP合成 + record用の素の2本）
bash scripts/start-collection-cameras.sh

# 別端末（Thor）: sender 起動
~/projects/XRoboToolkit-Orin-Video-Sender/OrinVideoSender --listen 0.0.0.0:13579 --device /dev/video20

# PICO: XRoboToolkit アプリで Remote Vision -> Listen -> ThorのIP:13579 -> Confirm

# 操作側PC: リーダー起動（右腕。左は run_leader_agent_left.sh）
THOR_IP=<ThorのIP> bash scripts/run_leader_agent_right.sh

# Thor: フォロワー起動（リーダー起動後。左は run_teleop_remote_left.sh）
bash scripts/run_teleop_remote_right.sh
```

### B. テレオペによるエピソード収集

前提: §A の手順1（カメラ分配）と手順4（操作側の leader agent）を稼働させたまま、手順5の `run_teleop_remote_*.sh` は止めてから実行する（`record_remote.sh` が follower 駆動を兼ねるため、動かしたままだと `can_fol_r` の openarm_follower 掴み合い・ZMQ ポート 5701/echo 5711 の衝突で落ちる）。

```bash
# Thor: X転送不要（素の ssh -t から）
env -u DISPLAY -u WAYLAND_DISPLAY OPERATOR=<名前> SCENE=<scenes/のname> \
  bash scripts/record_remote.sh <scene-tag>
```

キー操作: 右矢印 = 次のエピソード開始 / `a` = 両腕を開始姿勢へ復帰 / 左矢印 = 直前のテイクを破棄 / `esc` = 収集終了。

収集後、軽量メタ（`command.sh`/`manifest.json`/`console.log`/`ledger.csv` 等。データセット本体は含まない）を Vault へ回収する（Vault のある Vector/Mac 側で実行。Thor へ ssh して rsync するのでこのコマンド自体は Thor 上では打たない）:

```bash
bash scripts/pull_run.sh <UTC>-<scene-tag>
```

Thor 側の openarm-orboh/lerobot の実配置がこのリポの前提（`~/projects/...`）と異なる場合は `THOR_OPENARM_DIR` / `THOR_LEROBOT_DIR` で上書きできる。

### C. 両腕収集（bimanual・3 眼・板橋 2026-09〜）

両腕（`bi_openarm_follower` / `bi_openarm_leader`）と 3 眼カメラ（chest D435i＋左右 wrist D405）でエピソードを録る経路。固定値は `config/bimanual-collect.env`（`TASK`・`TASK_SLUG`・`SCENE`・3 カメラのシリアル・`GRIPPER_KP` 等）に置き、入口スクリプトはモード・タグ・本数だけを受ける。技術手順の全体（収集→変換→FT→rollout）は `docs/bimanual-pipeline-runbook.md`。

前提: `sudo bash scripts/canup.sh` で固定名 4 本（`can_fol_l / can_fol_r / can_lead_l / can_lead_r`）が UP、`python scripts/openarm_motorcheck.py` で 4 腕 32 モータが実値で応答（未給電だと全 0 のまま `[OK]` が出るので値を見る）。モニター表示は `bash scripts/preview_on_monitor.sh start`（rerun ビューア＋3 眼プレビュー）。収集スクリプトは起動時にプレビューを自分で止める（RealSense は 1 台 1 プロセス）。

```bash
# Thor: 素の ssh -t から（X転送しない）
bash scripts/collect_bimanual.sh dry                 # 2 本・捨て名・SCENE 不要（配管と 'a' キーの確認）
GRIPPER_KP=12 bash scripts/collect_bimanual.sh gate  # 受入ゲート 10 本（<slug>-v1-s1）→ 止めて検査
GRIPPER_KP=12 bash scripts/collect_bimanual.sh session s2 [10]   # 続きのセッション
```

- `TASK` / `TASK_SLUG` が `TODO` のままだと起動を拒否する。`gate` と `session` は `SCENE=<scenes/のname>` も必須（`scenes/_TEMPLATE.yaml` をコピーし TODO を全部埋める。`python scripts/collect_manifest.py check-scene --scene-path scenes/<name>.yaml` が残り TODO 数を出す。1 本でも録ったあとは書き換えず `-v2` の新ファイルにする）
- タスクの台帳 `config/tasks/<TASK_SLUG>.yaml` があるタスク（2026-10-28〜 `takeei_wood`）は、`gate` / `session` の前に `python3 scripts/task.py check` が指示文・開始姿勢セット・シーンを台帳の今の条件の版と突き合わせ、食い違えば止める。`ISSUE=T1-L2` を渡すと課題コードも見る。名前（外に見せる ID・HF のデータセットとモデル）は `python3 scripts/task.py name <task> <課題コード>`。説明は `config/tasks/README.md`
- `GRIPPER_KP` は follower gripper（dm4310）の位置 kp。既定 25 のまま硬い物を握り込むとコイルが 100 ℃のラッチに達する（2026-09-21 実測 99 ℃）。積み木など硬い対象は 10〜12 を渡す
- キー操作は §B と同じ（右矢印 = 次のエピソード / `a` = leader 左→右→follower の順に開始姿勢へ復帰 / 左矢印 = 直前を破棄 / `esc` = 終了）
- 2026-10-17 追加（**実機未検証・既定は無効**。付けなければ従来どおり）: `START_POSE_HOLD=true`（`a` の後と起動直後に両腕を開始姿勢で保持し、leader を放せる。保持を解く前に leader のずれを読み `START_POSE_HANDOFF_DEG`＝既定 15° を超えていたら始めない）、`SAVE_ON_RETURN=true`（`a` の時点で前のテイクを保存。右矢印は開始だけ）、`START_COUNTDOWN_S=3`（右矢印の後に秒読みとベル）。動機は待機中に leader の手首が垂れて follower の J7 が過温ラッチした件。説明は `scripts/record_bimanual_openarm.sh` の「開始姿勢での保持」、試験手順は Vault `07Member's-free-space/Kento/00-Dev-Log/2026-10-06-record-hold-and-cue-implementation.md` §5
- 監視: `python scripts/damiao_feedback_log.py can_lead_r can_fol_r can_lead_l can_fol_l --out ~/collections/dmlog-<日時>.csv`（受信のみ・並走可・LOST/BACK と温度ピーク）、赤点滅は `python scripts/damiao_errcheck.py can_fol_r can_fol_l --seconds 3`（`--clear` で過温ラッチ解除）
- 回収は §B と同じ `bash scripts/pull_run.sh <UTC>-bi-<slug>-<s1…>`（Vector/Mac 側で実行）

### D. バイラテラル(力覚フィードバック)の手応え確認

位置づけ: データ収集(§B/§C のlerobotベース)とは別系統。力覚フィードバックの手応えをその場で確認するための単腕単独テストで、データは記録されない。

前提: `openarm_teleop/build/bilateral_control` が未ビルドなら `bash scripts/build_teleop.sh` でビルドしておく(ROS 2 distro は jazzy/humble を自動検出するよう対応済み)。

```bash
# 右腕
bash scripts/launch_bilateral.sh right_arm can_lead_r can_fol_r
# 左腕
bash scripts/launch_bilateral.sh left_arm can_lead_l can_fol_l
```

内部で `canup.sh` の CAN 健全性チェックを呼ぶため、不健全なら起動しない(§CAN 健全性ゲート章と同じ仕組み)。

安全上の注意:
- Follower の可動域を確保すること。
- 非常停止ボタンが入ったままだとハンドシェイクが失敗する(実機で発生済み)。
- Ctrl-C で止まらない場合がある(USB-CAN カーネルドライバの D-state ハング。このリポで既知の事象)。応答しない場合は4本のアーム電源を物理的に切ってから `pkill -9` でプロセスを掃除する。
- 2026-10-19、右腕・左腕それぞれで実機テスト済み(成功)。

## 配置規約（契約）

**全マシン `~/projects/<repo>` に固定。lerobot venv は `~/projects/lerobot/.venv` に固定。** `run_teleop_*.sh` / `run_teleop_remote_*.sh` / `run_leader_agent_*.sh` / `calibrate_openarm.sh` / `scripts/g4/run_rollout.sh` など、テレオペ・収集・ロールアウトの実行スクリプト群はこの配置をハードコードして動く（`bootstrap.sh` の `PROJECTS_DIR` はテスト用途専用の上書きで、本番運用ではこの契約を守ること）。

## CAN 識別方式（なぜポート非依存か）

**別PCで `git clone` → `bash scripts/bootstrap.sh` を1回 → PEAK アダプタをどの USB ポートに挿しても leader/follower が逆転しない。** 「ケーブルを正しいポートに挿す」必要をなくすのが CAN 固定名運用のゴール。

leader/follower は **アダプタに焼いた channel_id**（役割タグ）＋ **dev_id**（チャンネル）で固定する。USB ポートには依存しない。

- channel_id（アダプタ NVRAM・抜き差し/別 PC でも保持）: `44444444`=Leader / `22222222`=Follower
- dev_id（チャンネル内在）: `0x0`=右(R) / `0x1`=左(L)
- → 固定名: `can_lead_r` / `can_lead_l` / `can_fol_r` / `can_fol_l`

固定名を付ける経路は2つ（併用）:

- **udev ルール** … `udev/80-openarm-can.rules` を `install_can_rules.sh`（`scripts/bootstrap.sh` の `setup.sh` 内で導入）。挿すと自動で固定名（PC ごと1回）。
- **canup.sh** … 実行時に channel_id を読んでリネーム。udev 未導入や hotplug 競合時の確実な保険。teleop 起動ラッパが内部で呼ぶ。

> **udev ルールは各 PC の `/etc/udev/rules.d/` に置くシステムファイルで、リポには含まれない。** リポを `git clone` / `git pull` しても各 PC の `/etc` は自動更新されない。**新規 PC も、既に運用中の PC でルールを更新する場合も、そのPCで1回 `sudo bash scripts/install_can_rules.sh` を実行して反映する**（旧ルールがあれば置換される）。実行後は `cat /etc/udev/rules.d/80-openarm-can.rules` で channel_id 版になったか確認できる。

> 逆転・流動的に崩れる根本原理（カーネル採番揺れ / udev がアダプタを識別できない / USB ポートキーの限界）は [docs/can-naming-principle.md](docs/can-naming-principle.md) を参照。

**固定名は teleop 専用ではなく、CAN を使う全タスク共通の土台。** モータ診断・データ収録・グリッパ較正・将来の WBC/policy 実機デプロイなど、どれも同じ4本のバスを開く。**新規スクリプトは生の `can0..3` を決め打ちせず、必ず固定名（`can_lead_r` 等）で参照すること**（生名は USB 認識順で揺れる）。`canup.sh`/udev で固定名を付けた後は、どのタスクも名前を書くだけで正しい物理アームに当たる。

> 補足: 2種類の ID は別物。**channel_id**＝アダプタ（どの役割/アーム）、**モータ ID `0x01..0x08`**＝バス上のどの関節（J1..J7+グリッパ、各モータに固定）。正しいバスを固定名で開けば、関節はモータ ID で正しく当たる。

## CAN 健全性ゲート（admin UP ≠ 通信可）

`ip link set up` は CAN コントローラが BUS-OFF / ERROR-PASSIVE でも admin UP を返す。`ip -br link` の "UP" 表示は「通信できる」ことを意味しない（右系統コネクタ半挿しで ERROR-PASSIVE のまま teleop が起動して落ちた既往がある）。

そこで `canup.sh` は bring-up 後に `can_check.sh` を呼び、`ip -details link show` の **CAN protocol state** で健全性を判定する。

- `ERROR-ACTIVE` / `ERROR-WARNING` = 健全
- `ERROR-PASSIVE` / `BUS-OFF` / `STOPPED` / IF 不在 = 異常（`canup.sh` は exit 1）

`launch_unilateral.sh` / `launch_bimanual.sh` は `canup.sh` の失敗で teleop 起動を中止する（`canup || exit 1`）。**CAN が通信可能でなければ、アームを動かす前に止まる。**

```bash
bash scripts/can_check.sh                       # 既定4本を判定（read-only）
bash scripts/can_check.sh can_lead_r can_fol_r  # 指定のみ
```

`can_check.sh` は state を読むだけなので、teleop 中に系統が落ちた後の事後診断にも単独で使える。注: Jetson Thor の onboard mttcan（`can4-7`）は OpenArm と無関係（常時 STOPPED）で対象外。

## 新しいアダプタの役割タグを焼く / 変える

channel_id は NVRAM 設定で**いつでも変更可**（後勝ち・別 PC でも保持）。

```bash
sudo bash scripts/burn_channel_id.sh <iface> leader      # 44444444 を焼く
sudo bash scripts/burn_channel_id.sh <iface> follower    # 22222222
```

値を変えたら `udev/80-openarm-can.rules` と `scripts/canup.sh` の channel_id も合わせること。

## モータ状態の確認（motorcheck / motor_status）

CAN（バス）が健全でも、各モータ（関節）の応答・角度は別レイヤー。確認用に2本ある。どちらも `openarm_can`（apt の `python3-openarm-can`）を使うので **システム python（`/usr/bin/python3`）で実行**。enable のみ・指令なし＝トルクは入らず腕は手で動かせる。**teleop が同じバスを使用中は実行しない**（制御が競合）。先に `sudo bash scripts/canup.sh` で固定名 IF を UP しておく。

- **`openarm_motorcheck.py`（一括点検）**: 全4アームを一発で enable→読取→disable し、関節ごとの表＋近零位判定（✓/✗）。teleop 前の健全確認や較正要否のスクリーニングに。
- **`openarm_motor_status.py <iface>`（ライブ監視）**: 単一アームを連続表示（度数）。`--hz N` で更新頻度、`--once` で1回。較正中・デバッグ中に特定関節を目で追う／応答確認に。

### 較正が要るかの読み方（motorcheck）

near-zero 判定は **「基準零位姿勢」**（各リンクをまっすぐ下げ鉛直に揃えた、較正時に NVRAM 零位として焼いた姿勢）に**保持して読む**のが前提。

- 基準姿勢で全関節 ✓（≈0）なら較正 OK。
- 基準姿勢に保持しているのに ✗（`|角度| > 5.7°`）の関節は、焼いてある零位が真の基準からズレている＝**再較正候補**。
- **任意の姿勢で読むと無意味**（非ゼロは単に今曲がっているだけ）。これはスクリーニングであって較正そのものではない。零点を決め直すのは calibrate 手順（Enter でその時の姿勢を NVRAM 零位に焼く）。

## スクリプト一覧

- `bootstrap.sh` … 1回の clone からテレオペ・収集ができる状態までの一括セットアップ（兄弟リポ clone・lerobot venv・sender ビルド・`setup.sh`）
- `setup.sh` … openarm-orboh 自身の一括セットアップ（submodule 取得・依存・ビルド・URDF・udev ルール）。`bootstrap.sh` から呼ばれる／単体実行も可
- `install_deps.sh` / `build_teleop.sh` / `gen_urdf.sh` … 依存・ビルド・URDF 生成
- `canup.sh` … channel_id+dev_id で CAN を固定名にして UP（ポート非依存・hotplug競合の保険）＋ `can_check.sh` で実 CAN state を検証し、不健全なら exit 1
- `can_check.sh` … CAN protocol state（ERROR-ACTIVE/WARNING=健全、BUS-OFF/ERROR-PASSIVE/STOPPED/不在=異常）で健全性を判定。admin UP の嘘を排除。canup が内部で呼ぶ／teleop 後の単独診断も可（read-only）
- `install_can_rules.sh` … udev 固定名ルール導入（PCごと1回）
- `burn_channel_id.sh` … アダプタ役割タグの焼き込み/変更
- `launch_unilateral.sh` / `launch_bimanual.sh` … ローカル teleop 起動（内部で canup.sh を呼ぶ。CAN が不健全なら teleop を起動せず停止）
- `launch_bilateral.sh` … 単腕バイラテラル(力覚フィードバック)起動・手応え確認専用（データ収集パイプラインとは独立。内部で canup.sh を呼ぶ）
- `run_leader_agent_left.sh` / `run_leader_agent_right.sh` … 遠隔テレオペの操作側 PC: リーダー実機を読み follower ホストへ ZMQ 送信（重力補償含む）
- `run_teleop_remote_left.sh` / `run_teleop_remote_right.sh` … 遠隔テレオペの follower ホスト（Thor）: ネットワーク越しのリーダー角度でフォロワーを追従
- `start-collection-cameras.sh` … 収集用カメラ分配（Thor）: 実カメラ2台を仮想カメラ3本（PICO用PiP合成 + record用の素の2本）へ分配
- `record_pick_place_openarm.sh` … pick-and-place データセット収録本体（`REMOTE=true`/`WRIST=true` 等は env で切替）
- `record_remote.sh` … 遠隔テレオペ収集の起動ラッパー（`record_pick_place_openarm.sh` に定型 env を焼き込み）
- `pull_run.sh` … 収集セッションの軽量メタを Thor から Vault へ回収
- `task.py list|check|name` … タスクの台帳（`config/tasks/`）の一覧・収集の前の突き合わせ・命名則どおりの名前（標準ライブラリだけ）
- `calibrate_openarm.sh` … OpenArm 恒久ゼロ較正（`hang_down`/`bump_to_stop`）
- `sync_status.sh` … openarm-orboh / lerobot / XRoboToolkit-Orin-Video-Sender 3リポ×両マシンの同期負債（未push/上流より古い/未コミット）を一括確認
- `openarm_motor_status.py <iface> [--hz N | --once]` … 単一アームのモータ状態ライブモニタ（関節角を度数表示・enable のみ＝トルク無しの read-only）
- `openarm_motorcheck.py [--only ROLE|IF,...]` … 全4アームのモータ health＋近零位較正チェック（固定名直接・can_map.env 不要）
- `install_gr00t_embodiment.sh` … GR00T（Isaac-GR00T）clone への `openarm_embodiment` 配置（学習・推論側。詳細は [gr00t/README.md](gr00t/README.md)）
- `scripts/_archive/` … bring-up 期の使い捨て診断（旧 can0-3 / can_map.env 前提・現行運用では非推奨）

## 開発者向け

**ブランチ運用**: `main` を唯一の配布・実行正本とする。`bootstrap.sh`・README・runbook は `main` だけを指す。`feat/`/`fix/` ブランチは1テーマ・短命とし、動作確認が済み次第すみやかに `main` へ merge して削除する。兄弟リポジトリ（lerobot・sender）の正本ブランチは `scripts/bootstrap.sh` 冒頭の設定ブロック（`LEROBOT_BRANCH`/`SENDER_BRANCH`）1箇所だけで管理し、変更時はそこを直してコミットする。

**同期確認**: 3リポ×両マシンの同期負債は `bash scripts/sync_status.sh`（`REMOTE=thor` でリモートも read-only ssh 確認）で一括で見る。

## 周知事項

- **G4 ロールアウト（`scripts/g4/run_rollout.sh`）は `--lang_instruction` の指定が必須。** 未指定は即 `exit 2` で止まる（過去のボトル/テープ文言取り違え事故の再発防止のための意図した仕様。旧コマンドをそのまま使い回すと落ちる）。

## トラブルシュート

- 固定名が付かない → `sudo bash scripts/canup.sh`（hotplug競合の保険）。`ip -br link show type can` で確認。
- `[skip] 未知の channel_id` … そのアダプタに役割タグが未焼き → `burn_channel_id.sh` で焼く。
- teleop 起動時に `Arm motor count` が7未満 → 該当アームを power cycle / コネクタ再嵌合（接触不良）。
- teleop が起動前に止まる / `canup.sh` が exit 1 → CAN state 異常（BUS-OFF / ERROR-PASSIVE）。`bash scripts/can_check.sh` の出力で該当バスを特定し、コネクタ再嵌合・PEAK 再挿し・`canup.sh` 再実行。
- `bootstrap.sh` が WARN を出す → checkout/reset/pull/削除/apt は自動実行しない設計なので、WARN 行に印字されたコマンドで手で直してから再実行する。
