#!/usr/bin/env bash
# unilateral teleop 起動（openarm-orboh / channel_id 固定名・リポ相対）
#   使い方: bash scripts/launch_unilateral.sh [arm_side] [leader_can] [follower_can]
#   既定 : right_arm can_lead_r can_fol_r
# ⚠️ 両アームが ENABLE され Follower は Leader に追従して動く。
#   Follower の可動域確保 / Leader・Follower を似姿勢に / 片手は Ctrl-C・電源カットへ。
set -uo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
ARM_SIDE=${1:-right_arm}
LEADER_CAN=${2:-can_lead_r}
FOLLOWER_CAN=${3:-can_fol_r}
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
BIN="$REPO/openarm_teleop/build/unilateral_control"

[ -f "$URDF" ] || { echo "[ERROR] URDF 無し: $URDF（scripts/setup.sh か scripts/gen_urdf.sh）"; exit 1; }
[ -x "$BIN" ]  || { echo "[ERROR] バイナリ無し: $BIN（scripts/setup.sh か scripts/build_teleop.sh）"; exit 1; }
[[ "$ARM_SIDE" == right_arm || "$ARM_SIDE" == left_arm ]] || { echo "[ERROR] arm_side は right_arm/left_arm"; exit 1; }

# CAN を channel_id 固定名で UP（USBポート非依存）
echo "[INFO] CAN bring-up + 健全性チェック (canup.sh)"
sudo bash "$REPO/scripts/canup.sh" || { echo "[ERROR] CAN bring-up/健全性チェック失敗。teleop を開始しない。"; exit 1; }

ROS_SETUP=""
for distro in jazzy humble; do
  [ -f "/opt/ros/$distro/setup.bash" ] && { ROS_SETUP="/opt/ros/$distro/setup.bash"; break; }
done
set +u; [ -n "$ROS_SETUP" ] && source "$ROS_SETUP"; set -u

cat <<EOF

================= UNILATERAL TELEOP 起動確認 =================
  arm_side : $ARM_SIDE
  LEADER   : $LEADER_CAN  (人が手で動かす側)
  FOLLOWER : $FOLLOWER_CAN  (自動追従して動く側 ← 可動域を確保)
------------------------------------------------------------
  両アームが ENABLE され Follower は即 Leader 姿勢へ追従します。
  Leader/Follower を似た姿勢に合わせ周囲の安全を確認。中断は Ctrl-C。
============================================================
EOF
read -r -p ">>> 準備OKなら Enter（中止は Ctrl-C）: " _

cd "$REPO/openarm_teleop"   # config/*.yaml を相対参照するため
echo "[INFO] launching unilateral_control ..."
"$BIN" "$URDF" "$URDF" "$ARM_SIDE" "$LEADER_CAN" "$FOLLOWER_CAN"
