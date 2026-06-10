#!/usr/bin/env bash
# PEAK PCAN-USB Pro FD のアダプタに channel_id（役割タグ）を焼く/変更する。
#
# 性質: NVRAM 書き込み。後勝ち（last-write-wins）・再起動/別PC でも保持・いつでも変更可。
#       このモデルは per-device（アダプタ単位）保持なので、アダプタの1チャンネルに焼けば両ch同値になる。
#       R/L は dev_id（0x0/0x1）で区別するので、channel_id はアダプタの leader/follower だけ決めればよい。
#
# 使い方:
#   sudo bash scripts/burn_channel_id.sh <interface> <leader|follower|8桁HEX>
#   例: sudo bash scripts/burn_channel_id.sh can0 leader      # 44444444 を焼く
#       sudo bash scripts/burn_channel_id.sh can2 follower    # 22222222
#       sudo bash scripts/burn_channel_id.sh can0 deadbeef    # 任意の8桁HEX
# 焼いた後: scripts/canup.sh か再起動で固定名が付く。値を変えたら udev/80-openarm-can.rules も合わせる。
#
# 出典: Dev-Log/Kento/2026-06-06-openarm-teleop-debug.md §1(a)（ethtool -E での channel_id 書き込み）。
set -euo pipefail
IF="${1:?interface（例 can0）を指定}"
ROLE="${2:?leader|follower|8桁HEX を指定}"

case "$ROLE" in
  leader)   VAL="44444444" ;;
  follower) VAL="22222222" ;;
  *) [[ "$ROLE" =~ ^[0-9a-fA-F]{8}$ ]] || { echo "[err] role は leader / follower / 8桁HEX"; exit 1; }; VAL="$ROLE" ;;
esac

[[ $EUID -ne 0 ]] && exec sudo "$0" "$@"
ip link show "$IF" >/dev/null 2>&1 || { echo "[err] interface 不在: $IF"; exit 1; }

echo "[burn] $IF <- channel_id=$VAL（NVRAM書き込み・後で変更可）"
read -r -p ">>> 実行してよければ Enter（中止は Ctrl-C）: " _

# 4バイトを raw 化して ethtool -E に流す（Dev-Log §1a の実証手順）
echo "$VAL" | sed 's/../& /g' | xxd -r -p | ethtool -E "$IF" || {
  echo "[err] ethtool -E 失敗。ドライバ/オフセット差異の可能性 → Dev-Log §1a を参照して手動書き込み"; exit 1; }

echo "[ok] 焼き込み完了。read-back（再読込/再起動後に反映）:"
cat "/sys/class/net/$IF/peak_usb/can_channel_id" 2>/dev/null || echo "(後で ip link 再読込 or 再起動で確認)"
