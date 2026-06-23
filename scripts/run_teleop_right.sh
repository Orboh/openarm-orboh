#!/usr/bin/env bash
# 右腕 lerobot teleop（重力補償＋vel FF＋fps200）。停止は Ctrl-C。
# 使い方: bash scripts/run_teleop_right.sh [scale]   （例: ... 1.0）
# 前提: lerobot venv は ~/projects/lerobot/.venv（Vector/Thor 共通）。URDF はリポ相対参照。
SCALE="${1:-1.2}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
BIN="$HOME/projects/lerobot/.venv/bin/lerobot-teleoperate"
"$BIN" \
  --robot.type=openarm_follower --robot.port=can_fol_r --robot.side=right --robot.id=follower_right \
  --teleop.type=openarm_leader --teleop.port=can_lead_r --teleop.id=leader_right \
  --teleop.gravity_compensation=true \
  --teleop.gravity_urdf_path="$URDF" \
  --teleop.gravity_side=right \
  --teleop.gravity_scale="$SCALE" \
  --teleop.use_velocity_and_torque=true \
  --fps=200
