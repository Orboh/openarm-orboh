#!/usr/bin/env bash
# 両腕 bimanual teleop（右: can_lead_r->can_fol_r / 左: can_lead_l->can_fol_l）。リポ相対・channel_id 固定名。
#   1つの Ctrl-C で両方停止 → 全4bus を disable。
# ⚠️ 既存 teleop が動いていないこと。両ペアとも Leader/Follower を似姿勢に、両 Follower の可動域を確保。
set -uo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
BIN="$REPO/openarm_teleop/build/unilateral_control"

[ -f "$URDF" ] && [ -x "$BIN" ] || { echo "[ERROR] URDF/バイナリ不在（scripts/setup.sh で生成）"; exit 1; }

echo "=== 既存 teleop プロセス確認 ==="
if pgrep -af "unilateral_control|bilateral_control" | grep -v $$ ; then
  echo "[!] teleop が既に動作中。先に停止してから再実行。"; exit 1
fi

echo "=== CAN bring-up (canup.sh) ==="
sudo bash "$REPO/scripts/canup.sh"

set +u; source /opt/ros/jazzy/setup.bash; set -u

cat <<EOF

============== BIMANUAL TELEOP（両腕同時）==============
  右: LEADER can_lead_r -> FOLLOWER can_fol_r
  左: LEADER can_lead_l -> FOLLOWER can_fol_l
  両 Follower(can_fol_r, can_fol_l) が動きます。可動域を確保し、
  4本とも Leader/Follower を似た姿勢に合わせてください。停止は Ctrl-C。
=======================================================
EOF
read -r -p ">>> 準備OKなら Enter（中止は Ctrl-C）: " _

cd "$REPO/openarm_teleop"
PIDS=()
cleanup() {
  echo; echo "[INFO] 停止: プロセス kill + 全bus disable"
  for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
  sleep 0.5
  for ch in can_lead_r can_lead_l can_fol_r can_fol_l; do sudo openarm-can-cli -i "$ch" disable >/dev/null 2>&1 || true; done
  echo "[INFO] 全モータ disable 完了"
}
trap cleanup INT TERM EXIT

echo "[INFO] 右腕 teleop 起動 (can_lead_r->can_fol_r)"
"$BIN" "$URDF" "$URDF" right_arm can_lead_r can_fol_r & PIDS+=($!)
sleep 1
echo "[INFO] 左腕 teleop 起動 (can_lead_l->can_fol_l)"
"$BIN" "$URDF" "$URDF" left_arm can_lead_l can_fol_l & PIDS+=($!)
echo "[INFO] 両腕 teleop 稼働中。停止は Ctrl-C。"
wait
