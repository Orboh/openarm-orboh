#!/usr/bin/env bash
# 左腕 lerobot teleop（重力補償＋vel FF＋fps200）。停止は Ctrl-C。
# 起動時整列: leader→follower が初期姿勢（肘36°）へ自動で動く（公式AdjustPosition移植・各約2.2秒・lerobot fork の align_on_connect=true 既定）。
# 使い方: bash scripts/run_teleop_left.sh [scale]   （過補償で上がるなら下げる例: ... 1.0）
# 前提: lerobot venv は ~/projects/lerobot/.venv（Vector/Thor 共通）。URDF はリポ相対参照。
SCALE="${1:-1.2}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
BIN="$HOME/projects/lerobot/.venv/bin/lerobot-teleoperate"
"$BIN" \
  --robot.type=openarm_follower --robot.port=can_fol_l --robot.side=left --robot.id=follower_left \
  --teleop.type=openarm_leader --teleop.port=can_lead_l --teleop.id=leader_left \
  --teleop.gravity_compensation=true \
  --teleop.gravity_urdf_path="$URDF" \
  --teleop.gravity_side=left \
  --teleop.gravity_scale="$SCALE" \
  --teleop.use_velocity_and_torque=true \
  --fps=200
