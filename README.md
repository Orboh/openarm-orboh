# openarm-orboh

OpenArm（enactic）の teleop を Orboh 運用向けにまとめたリポジトリ。submodule（`openarm_teleop` / `openarm_description`）＋ CAN 固定名運用 ＋ 起動スクリプト一式。

## 設計目標（理想）

**別PCで `git clone` → `scripts/setup.sh` を1回 → PEAK アダプタをどのUSBポートに挿しても leader/follower が逆転しない。** 「ケーブルを正しいポートに挿す」必要をなくすのがゴール。

## クイックスタート

```bash
git clone --recurse-submodules <repo-url> openarm-orboh
cd openarm-orboh
bash scripts/setup.sh                       # submodule取得・依存・ビルド・URDF・udevルール（sudo含む・PCごと1回）

# PEAK 2台を直挿し（ポート不問・ハブNG）
sudo bash scripts/canup.sh                  # CAN を固定名にして UP

bash scripts/launch_unilateral.sh right_arm can_lead_r can_fol_r   # 右腕 teleop
# 両腕同時: bash scripts/launch_bimanual.sh        停止: Ctrl-C（全 bus disable）
```

> **初回のみ**: 役割タグ未設定（新品）のアダプタは先に焼く → `sudo bash scripts/burn_channel_id.sh <iface> leader`（or `follower`）。既に焼いてあるアダプタはそのままでよい（channel_id はアダプタ NVRAM に保持され、別PCでも残る）。

## CAN 識別方式（なぜポート非依存か）

leader/follower は **アダプタに焼いた channel_id**（役割タグ）＋ **dev_id**（チャンネル）で固定する。USBポートには依存しない。

- channel_id（アダプタ NVRAM・抜き差し/別PCでも保持）: `44444444`=Leader / `22222222`=Follower
- dev_id（チャンネル内在）: `0x0`=右(R) / `0x1`=左(L)
- → 固定名: `can_lead_r` / `can_lead_l` / `can_fol_r` / `can_fol_l`

固定名を付ける経路は2つ（併用）:

- **udev ルール** … `udev/80-openarm-can.rules` を `install_can_rules.sh` で導入。挿すと自動で固定名（PCごと1回）。
- **canup.sh** … 実行時に channel_id を読んでリネーム。udev 未導入や hotplug 競合時の確実な保険。teleop 起動ラッパが内部で呼ぶ。

> **udev ルールは各PCの `/etc/udev/rules.d/` に置くシステムファイルで、リポには含まれない。** リポを `git clone` / `git pull` しても各PCの `/etc` は自動更新されない。**新規PCも、既に運用中のPCでルールを更新する場合も、そのPCで1回 `sudo bash scripts/install_can_rules.sh` を実行して反映する**（旧ルールがあれば置換される）。実行後は `cat /etc/udev/rules.d/80-openarm-can.rules` で channel_id 版になったか確認できる。

> 逆転・流動的に崩れる根本原理（カーネル採番揺れ / udev がアダプタを識別できない / USBポートキーの限界）は [docs/can-naming-principle.md](docs/can-naming-principle.md) を参照。

**固定名は teleop 専用ではなく、CAN を使う全タスク共通の土台。** モータ診断・データ収録・グリッパ較正・将来の WBC/policy 実機デプロイなど、どれも同じ4本のバスを開く。**新規スクリプトは生の `can0..3` を決め打ちせず、必ず固定名（`can_lead_r` 等）で参照すること**（生名は USB認識順で揺れる）。`canup.sh`/udev で固定名を付けた後は、どのタスクも名前を書くだけで正しい物理アームに当たる。

> 補足: 2種類のIDは別物。**channel_id**＝アダプタ（どの役割/アーム）、**モータID `0x01..0x08`**＝バス上のどの関節（J1..J7+グリッパ、各モータに固定）。正しいバスを固定名で開けば、関節はモータIDで正しく当たる。

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

channel_id は NVRAM 設定で**いつでも変更可**（後勝ち・別PCでも保持）。

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

- 基準姿勢で全関節 ✓（≈0）なら較正OK。
- 基準姿勢に保持しているのに ✗（`|角度| > 5.7°`）の関節は、焼いてある零位が真の基準からズレている＝**再較正候補**。
- **任意の姿勢で読むと無意味**（非ゼロは単に今曲がっているだけ）。これはスクリーニングであって較正そのものではない。零点を決め直すのは calibrate 手順（Enter でその時の姿勢を NVRAM 零位に焼く）。

## スクリプト一覧

- `setup.sh` … clone 後の一括セットアップ
- `install_deps.sh` / `build_teleop.sh` / `gen_urdf.sh` … 依存・ビルド・URDF 生成
- `canup.sh` … channel_id+dev_id で CAN を固定名にして UP（ポート非依存・hotplug競合の保険）＋ `can_check.sh` で実 CAN state を検証し、不健全なら exit 1
- `can_check.sh` … CAN protocol state（ERROR-ACTIVE/WARNING=健全、BUS-OFF/ERROR-PASSIVE/STOPPED/不在=異常）で健全性を判定。admin UP の嘘を排除。canup が内部で呼ぶ／teleop 後の単独診断も可（read-only）
- `install_can_rules.sh` … udev 固定名ルール導入（PCごと1回）
- `burn_channel_id.sh` … アダプタ役割タグの焼き込み/変更
- `launch_unilateral.sh` / `launch_bimanual.sh` … teleop 起動（内部で canup.sh を呼ぶ。CAN が不健全なら teleop を起動せず停止）
- `openarm_motor_status.py <iface> [--hz N | --once]` … 単一アームのモータ状態ライブモニタ（関節角を度数表示・enable のみ＝トルク無しの read-only）
- `openarm_motorcheck.py [--only ROLE|IF,...]` … 全4アームのモータ health＋近零位較正チェック（固定名直接・can_map.env 不要）
- `scripts/_archive/` … bring-up 期の使い捨て診断（旧 can0-3 / can_map.env 前提・現行運用では非推奨）

## トラブルシュート

- 固定名が付かない → `sudo bash scripts/canup.sh`（hotplug競合の保険）。`ip -br link show type can` で確認。
- `[skip] 未知の channel_id` … そのアダプタに役割タグが未焼き → `burn_channel_id.sh` で焼く。
- teleop 起動時に `Arm motor count` が 7 未満 → 該当アームを power cycle / コネクタ再嵌合（接触不良）。
- teleop が起動前に止まる / `canup.sh` が exit 1 → CAN state 異常（BUS-OFF / ERROR-PASSIVE）。`bash scripts/can_check.sh` の出力で該当バスを特定し、コネクタ再嵌合・PEAK 再挿し・`canup.sh` 再実行。
