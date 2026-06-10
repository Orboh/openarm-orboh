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

## CAN 識別方式（なぜポート非依存か）

leader/follower は **アダプタに焼いた channel_id**（役割タグ）＋ **dev_id**（チャンネル）で固定する。USBポートには依存しない。

- channel_id（アダプタ NVRAM・抜き差し/別PCでも保持）: `44444444`=Leader / `22222222`=Follower
- dev_id（チャンネル内在）: `0x0`=右(R) / `0x1`=左(L)
- → 固定名: `can_lead_r` / `can_lead_l` / `can_fol_r` / `can_fol_l`

固定名を付ける経路は2つ（併用）:

- **udev ルール** … `udev/80-openarm-can.rules` を `install_can_rules.sh` で導入。挿すと自動で固定名（PCごと1回）。
- **canup.sh** … 実行時に channel_id を読んでリネーム。udev 未導入や hotplug 競合時の確実な保険。teleop 起動ラッパが内部で呼ぶ。

> 逆転・流動的に崩れる根本原理（カーネル採番揺れ / udev がアダプタを識別できない / USBポートキーの限界）は [docs/can-naming-principle.md](docs/can-naming-principle.md) を参照。

## 新しいアダプタの役割タグを焼く / 変える

channel_id は NVRAM 設定で**いつでも変更可**（後勝ち・別PCでも保持）。

```bash
sudo bash scripts/burn_channel_id.sh <iface> leader      # 44444444 を焼く
sudo bash scripts/burn_channel_id.sh <iface> follower    # 22222222
```

値を変えたら `udev/80-openarm-can.rules` と `scripts/canup.sh` の channel_id も合わせること。

## スクリプト一覧

- `setup.sh` … clone 後の一括セットアップ
- `install_deps.sh` / `build_teleop.sh` / `gen_urdf.sh` … 依存・ビルド・URDF 生成
- `canup.sh` … channel_id+dev_id で CAN を固定名にして UP（ポート非依存・hotplug競合の保険）
- `install_can_rules.sh` … udev 固定名ルール導入（PCごと1回）
- `burn_channel_id.sh` … アダプタ役割タグの焼き込み/変更
- `launch_unilateral.sh` / `launch_bimanual.sh` … teleop 起動（内部で canup.sh を呼ぶ）

## トラブルシュート

- 固定名が付かない → `sudo bash scripts/canup.sh`（hotplug競合の保険）。`ip -br link show type can` で確認。
- `[skip] 未知の channel_id` … そのアダプタに役割タグが未焼き → `burn_channel_id.sh` で焼く。
- teleop 起動時に `Arm motor count` が 7 未満 → 該当アームを power cycle / コネクタ再嵌合（接触不良）。
