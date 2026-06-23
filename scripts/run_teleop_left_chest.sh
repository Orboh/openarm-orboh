#!/usr/bin/env bash
# 左腕 teleop + chest カメラ表示（D435i のみ・単眼）。停止は Ctrl-C。
# カメラ込みなのでループは ~30fps（カメラ律速）。純follow感だけ見たいなら
# scripts/run_teleop_left.sh（fps200・カメラ無し）を使う。
# rerunビューアが立つ環境で実行すること（Thorのローカル画面 or rerun connect。
# 完全ヘッドレスSSHで viewer が出ないときは末尾を --display_data=false にする）。
# 使い方: bash scripts/run_teleop_left_chest.sh [scale]      （過補償で上がるなら例: ... 1.0）
# 前提: lerobot venv は ~/projects/lerobot/.venv（Vector/Thor 共通）。URDF はリポ相対参照。
SCALE="${1:-1.2}"
FPS="${FPS:-30}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
BIN="$HOME/projects/lerobot/.venv/bin/lerobot-teleoperate"
"$BIN" \
  --robot.type=openarm_follower --robot.port=can_fol_l --robot.side=left --robot.id=follower_left \
  --robot.cameras="{ chest: {type: intelrealsense, serial_number_or_name: '344422070898', width: 640, height: 480, fps: $FPS} }" \
  --teleop.type=openarm_leader --teleop.port=can_lead_l --teleop.id=leader_left \
  --teleop.gravity_compensation=true \
  --teleop.gravity_urdf_path="$URDF" \
  --teleop.gravity_side=left \
  --teleop.gravity_scale="$SCALE" \
  --teleop.use_velocity_and_torque=true \
  --display_data=true \
  --fps="$FPS"
