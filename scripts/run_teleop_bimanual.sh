#!/usr/bin/env bash
# 両腕 bimanual lerobot teleop（重力補償＋vel FF＋fps200）。停止は Ctrl-C。
# 使い方: bash scripts/run_teleop_bimanual.sh [left_scale] [right_scale]  （既定 両腕1.2。右だけ下げる例: ... 1.2 1.0）
# ⚠ 起動時は両腕を真下に垂らす（零位が焼かれる）。右系統CANはbus-off既往。
# NOTE: leader 側は左右とも gravity_side を明示する（未指定だと既定 left＝右腕の重力補償が左モデルで誤計算される。2026-06-23 修正）。
# 前提: lerobot venv は ~/projects/lerobot/.venv（Vector/Thor 共通）。URDF はリポ相対参照。
LSCALE="${1:-1.2}"
RSCALE="${2:-$LSCALE}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
BIN="$HOME/projects/lerobot/.venv/bin/lerobot-teleoperate"
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
  --fps=200
