#!/usr/bin/env bash
# bilateral teleop 起動（力覚フィードバック版・openarm-orboh / channel_id 固定名・リポ相対）
#   使い方: bash scripts/launch_bilateral.sh [arm_side] [leader_can] [follower_can]
#   既定 : right_arm can_lead_r can_fol_r
# これは手応え確認用の単腕テストであり、データ収集(lerobot/collect_bimanual.sh)とは別系統。
# ⚠️ 両アームが ENABLE され、Follower は Leader に追従しつつ Leader 側にも力が返る。
#   Follower の可動域確保 / Leader・Follower を似姿勢に / 中断は Ctrl-C。
set -uo pipefail
REPO=$(cd "$(dirname "$0")/.." && pwd)
ARM_SIDE=${1:-right_arm}
LEADER_CAN=${2:-can_lead_r}
FOLLOWER_CAN=${3:-can_fol_r}
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
BIN="$REPO/openarm_teleop/build/bilateral_control"

[ -f "$URDF" ] || { echo "[ERROR] URDF 無し: $URDF（scripts/setup.sh か scripts/gen_urdf.sh）"; exit 1; }
[ -x "$BIN" ]  || { echo "[ERROR] バイナリ無し: $BIN（scripts/build_teleop.sh でビルド）"; exit 1; }
[[ "$ARM_SIDE" == right_arm || "$ARM_SIDE" == left_arm ]] || { echo "[ERROR] arm_side は right_arm/left_arm"; exit 1; }

echo "[INFO] CAN bring-up + 健全性チェック (canup.sh)"
sudo bash "$REPO/scripts/canup.sh" || { echo "[ERROR] CAN bring-up/健全性チェック失敗。teleop を開始しない。"; exit 1; }

set +u; for _d in jazzy humble; do [ -f "/opt/ros/$_d/setup.bash" ] && { source "/opt/ros/$_d/setup.bash"; break; }; done; set -u

cat <<EOF

============= BILATERAL TELEOP 起動確認（力覚フィードバック・手応え確認用） =============
  arm_side : $ARM_SIDE
  LEADER   : $LEADER_CAN  (人が手で動かす側。Follower の負荷が力として返ってくる)
  FOLLOWER : $FOLLOWER_CAN  (自動追従して動く側 ← 可動域を確保)
------------------------------------------------------------
  データ収集(lerobot)用ではなく、手応え確認のための単独テストです。
  両アームが ENABLE され Follower は Leader 姿勢へ追従します。
  Leader/Follower を似た姿勢に合わせ周囲の安全を確認。中断は Ctrl-C。
========================================================================
EOF
read -r -p ">>> 準備OKなら Enter（中止は Ctrl-C）: " _

cd "$REPO/openarm_teleop"   # config/*.yaml を相対参照するため
echo "[INFO] launching bilateral_control ..."
"$BIN" "$URDF" "$URDF" "$ARM_SIDE" "$LEADER_CAN" "$FOLLOWER_CAN"
