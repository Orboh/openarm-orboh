#!/usr/bin/env bash
# channel_id ベースの CAN 固定名 udev ルールを /etc/udev/rules.d に導入（PCごと1回）。
# 導入後はアダプタを抜き差し or 再起動すると channel_id に基づく固定名が付く（ポート不問）。
set -euo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
[[ $EUID -ne 0 ]] && exec sudo "$0" "$@"

install -m 0644 "$REPO/udev/80-openarm-can.rules" /etc/udev/rules.d/80-openarm-can.rules
udevadm control --reload
echo "[ok] 導入: /etc/udev/rules.d/80-openarm-can.rules"
echo "    アダプタ抜き差し（または再起動）→ 'ip -br link show type can' で can_lead_*/can_fol_* を確認。"
echo "    hotplug 直後に名前が付かない時は: sudo bash $REPO/scripts/canup.sh"
