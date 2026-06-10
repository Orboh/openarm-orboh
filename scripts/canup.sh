#!/usr/bin/env bash
# OpenArm CAN bring-up（channel_id + dev_id ベース = USBポート非依存）
#
# 各 PEAK チャンネルを、アダプタに焼かれた channel_id（役割タグ）と dev_id（R/L）で
# 固定名にリネーム → CAN-FD 設定 → UP する。どのUSBポートに挿しても leader/follower は固定。
# udev ルールが未導入/hotplug競合で空振りした環境でも、これを実行すれば確実に固定名が付く。
# 名前が完全にスワップした状態（旧ポートキー rule が逆に付けた等）でも、temp名経由の2パスで衝突なく直す。
#
# 役割タグ（アダプタ NVRAM の channel_id。scripts/burn_channel_id.sh で焼く）:
#   44444444 = Leader アダプタ / 22222222 = Follower アダプタ
#   dev_id 0x0 = 右(R) / 0x1 = 左(L)
# 使い方: sudo bash scripts/canup.sh
set -euo pipefail

# channel_id:dev_id -> 固定名（値を変えたら udev/80-openarm-can.rules も合わせる）
declare -A ROLE=(
  ["44444444:0x0"]="can_lead_r"
  ["44444444:0x1"]="can_lead_l"
  ["22222222:0x0"]="can_fol_r"
  ["22222222:0x1"]="can_fol_l"
)

[[ $EUID -ne 0 ]] && exec sudo -E "$0" "$@"

# 1) PEAK CAN IF を収集（channel_id を持つネット IF）。現在名 -> 目標固定名 を決める。
declare -A CUR2TARGET
for sys in /sys/class/net/*; do
  [[ -e "$sys/peak_usb/can_channel_id" ]] || continue
  ifn=$(basename "$sys")
  cid=$(cat "$sys/peak_usb/can_channel_id" 2>/dev/null || echo "")
  did=$(cat "$sys/dev_id" 2>/dev/null || echo "")
  tgt="${ROLE[${cid}:${did}]:-}"
  if [[ -z "$tgt" ]]; then
    echo "[skip] $ifn: 未知 channel_id/dev_id ($cid / $did)。burn_channel_id.sh で焼いたか確認"
    continue
  fi
  CUR2TARGET["$ifn"]="$tgt"
done

# 2) 衝突回避パスA: 改名が要る IF を一旦 temp 名へ退避（最終名を全部空ける）
i=0
declare -A TMP2TARGET
for cur in "${!CUR2TARGET[@]}"; do
  tgt="${CUR2TARGET[$cur]}"
  [[ "$cur" == "$tgt" ]] && continue
  tmp="oatmp$i"; i=$((i+1))
  ip link set "$cur" down
  ip link set "$cur" name "$tmp"
  TMP2TARGET["$tmp"]="$tgt"
  echo "[stage]  $cur -> $tmp  (=> $tgt)"
done
# 3) 衝突回避パスB: temp -> 最終固定名
for tmp in "${!TMP2TARGET[@]}"; do
  tgt="${TMP2TARGET[$tmp]}"
  ip link set "$tmp" name "$tgt"
  echo "[rename] $tmp -> $tgt"
done

# 4) CAN-FD 設定 + UP（実証済み 1M/5M）。openarm-can-cli が無ければ ip link で代替。
up=0
for cur in "${!CUR2TARGET[@]}"; do
  tgt="${CUR2TARGET[$cur]}"
  if command -v openarm-can-cli >/dev/null 2>&1; then
    openarm-can-cli -i "$tgt" can_configure >/dev/null
  else
    ip link set "$tgt" down 2>/dev/null || true
    ip link set "$tgt" type can bitrate 1000000 sample-point 0.75 \
      dbitrate 5000000 dsample-point 0.75 dsjw 2 fd on restart-ms 100
    ip link set "$tgt" up
  fi
  up=$((up+1))
  cid=$(cat "/sys/class/net/$tgt/peak_usb/can_channel_id" 2>/dev/null || echo "?")
  port=$(basename "$(readlink -f "/sys/class/net/$tgt/device" 2>/dev/null)" 2>/dev/null || echo "?")
  echo "[up] $tgt  channel_id=$cid  port=$port"
done

echo
echo "up=$up  （channel_id が同じなら、port が変わっても固定名は不変＝ポート非依存）"
ip -br link show type can
[[ $up -ge 4 ]] || echo "[!] 4ch 揃っていない。PEAK 2台を直挿し（ハブNG）か、channel_id 焼き込みを確認"
