#!/usr/bin/env bash
# OpenArm CAN bring-up（channel_id + dev_id ベース = USBポート非依存）
#
# 各 PEAK チャンネルを、アダプタに焼かれた channel_id（役割タグ）と dev_id（R/L）で
# 固定名にリネーム → CAN-FD 設定 → UP する。どのUSBポートに挿しても leader/follower は固定。
# udev ルールが未導入/hotplug競合で空振りした環境でも、これを実行すれば確実に固定名が付く。
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

renamed=0; up=0
for SYS in /sys/class/net/can*; do
  [[ -e "$SYS" ]] || continue
  cur=$(basename "$SYS")
  cid=$(cat "$SYS/peak_usb/can_channel_id" 2>/dev/null || echo "")
  did=$(cat "$SYS/dev_id" 2>/dev/null || echo "")
  target="${ROLE[${cid}:${did}]:-}"
  if [[ -z "$target" ]]; then
    echo "[skip] $cur: 未知の channel_id/dev_id ($cid / $did)。burn_channel_id.sh で焼いたか確認"
    continue
  fi
  if [[ "$cur" != "$target" ]]; then
    echo "[rename] $cur -> $target (channel_id=$cid dev_id=$did)"
    ip link set "$cur" down
    ip link set "$cur" name "$target"
    renamed=$((renamed+1))
  fi
  # CAN-FD 設定 + UP（実証済みパラメータ 1M/5M）。openarm-can-cli が無ければ ip link で代替。
  if command -v openarm-can-cli >/dev/null 2>&1; then
    openarm-can-cli -i "$target" can_configure >/dev/null
  else
    ip link set "$target" down 2>/dev/null || true
    ip link set "$target" type can bitrate 1000000 sample-point 0.75 \
      dbitrate 5000000 dsample-point 0.75 dsjw 2 fd on restart-ms 100
    ip link set "$target" up
  fi
  up=$((up+1))
  echo "[up] $target (channel_id=$cid dev_id=$did)"
done

echo
echo "renamed=$renamed  up=$up"
ip -br link show type can
[[ $up -ge 4 ]] || echo "[!] 4ch 揃っていない。PEAK 2台を直挿し（ハブNG）か、channel_id 焼き込みを確認"
