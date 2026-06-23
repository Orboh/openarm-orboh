#!/usr/bin/env bash
# 左腕 teleop + chest(D435i) + wrist_left(D405) カメラ表示（2眼）。停止は Ctrl-C。
# カメラ2台でループ ~30fps（カメラ律速）。D405は色(RGB)で開く（record と同じ・深度は使わない）。
# rerunビューアが立つ環境で実行すること（完全ヘッドレスで出ないときは --display_data=false）。
# 使い方: bash scripts/run_teleop_left_chest_wrist.sh [scale]
# 注: D405は現状1台のみ接続・ベンチ確認段階。手首装着の左右が確定したらキー名 wrist_left/right を見直す。
# 前提: lerobot venv は ~/projects/lerobot/.venv（Vector/Thor 共通）。URDF はリポ相対参照。
SCALE="${1:-1.2}"
FPS="${FPS:-30}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
BIN="$HOME/projects/lerobot/.venv/bin/lerobot-teleoperate"
"$BIN" \
  --robot.type=openarm_follower --robot.port=can_fol_l --robot.side=left --robot.id=follower_left \
  --robot.cameras="{ chest: {type: intelrealsense, serial_number_or_name: '344422070898', width: 640, height: 480, fps: $FPS}, wrist_left: {type: intelrealsense, serial_number_or_name: '260322272302', width: 640, height: 480, fps: $FPS} }" \
  --teleop.type=openarm_leader --teleop.port=can_lead_l --teleop.id=leader_left \
  --teleop.gravity_compensation=true \
  --teleop.gravity_urdf_path="$URDF" \
  --teleop.gravity_side=left \
  --teleop.gravity_scale="$SCALE" \
  --teleop.use_velocity_and_torque=true \
  --display_data=true \
  --fps="$FPS"
