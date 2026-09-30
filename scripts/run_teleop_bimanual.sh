#!/usr/bin/env bash
# 両腕 bimanual lerobot teleop（重力補償＋vel FF＋fps200）。停止は Ctrl-C。
# 起動時整列: 各腕 leader→follower が初期姿勢（肘36°）へ自動で動く（公式AdjustPosition移植・各約2.2秒×4・lerobot fork の align_on_connect=true 既定）。
# 使い方: bash scripts/run_teleop_bimanual.sh [left_scale] [right_scale]  （既定 両腕1.0＝2026-09-22 Kento の運用値。旧既定 1.2。左だけ上げる例: ... 1.1 1.0）
# ⚠ 起動時は両腕を真下に垂らす（零位が焼かれる）。右系統CANはbus-off既往。
# NOTE: leader 側は左右とも gravity_side を明示する（未指定だと既定 left＝右腕の重力補償が左モデルで誤計算される。2026-06-23 修正）。
# 前提: lerobot venv は ~/projects/lerobot/.venv（Vector/Thor 共通）。URDF はリポ相対参照。
LSCALE="${1:-1.0}"
RSCALE="${2:-$LSCALE}"
# 3 番目以降の引数は lerobot-teleoperate へそのまま渡す（例: グリッパの kp を下げる
#   --robot.left_arm_config.position_kp=[240,240,240,240,24,31,25,10] --robot.right_arm_config.position_kp=[...]
#   2026-09-21: 木材把持で follower gripper がコイル過温ラッチ（kp 25 で握り込み→ストール電流）。kp を下げて飽和トルクを抑える）
EXTRA_ARGS=("${@:3}")
REPO="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
BIN="$HOME/projects/lerobot/.venv/bin/lerobot-teleoperate"
# 開始姿勢: 開始姿勢セット（config/pose_sets/<POSE_SET>/・未指定なら CURRENT）の left/right.yaml。
# セットに無い側の腕は公式既定(肘36°)にフォールバック。
source "$REPO/scripts/lib/pose_set.sh"
pose_set_load current || exit 1
pose_set_check_calib warn
POSE_ARGS=()
[ -f "$LPOSE" ] && POSE_ARGS+=(--robot.left_arm_config.initial_pose_path="$LPOSE" --teleop.left_arm_config.initial_pose_path="$LPOSE")
[ -f "$RPOSE" ] && POSE_ARGS+=(--robot.right_arm_config.initial_pose_path="$RPOSE" --teleop.right_arm_config.initial_pose_path="$RPOSE")
"$BIN" \
  --robot.type=bi_openarm_follower \
  --robot.left_arm_config.port=can_fol_l --robot.left_arm_config.side=left \
  --robot.right_arm_config.port=can_fol_r --robot.right_arm_config.side=right \
  --robot.id=follower \
  --teleop.type=bi_openarm_leader \
  --teleop.left_arm_config.port=can_lead_l \
  --teleop.right_arm_config.port=can_lead_r \
  --teleop.id=leader \
  --teleop.left_arm_config.gravity_compensation=true \
  --teleop.left_arm_config.gravity_urdf_path="$URDF" \
  --teleop.left_arm_config.gravity_side=left \
  --teleop.left_arm_config.gravity_scale="$LSCALE" \
  --teleop.left_arm_config.use_velocity_and_torque=true \
  --teleop.right_arm_config.gravity_compensation=true \
  --teleop.right_arm_config.gravity_urdf_path="$URDF" \
  --teleop.right_arm_config.gravity_side=right \
  --teleop.right_arm_config.gravity_scale="$RSCALE" \
  --teleop.right_arm_config.use_velocity_and_torque=true \
  --fps=200 \
  "${POSE_ARGS[@]}" \
  ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"}
