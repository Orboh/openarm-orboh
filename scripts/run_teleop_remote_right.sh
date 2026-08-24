#!/usr/bin/env bash
# 別室テレオペ（follower ホスト＝Thor 側）: ネットワーク越しのリーダー角度で右腕フォロワーを追従させる。
# 設計書= toyota-body-orboh vault "openarm-remote-teleop-design"。
# 挙動: 起動後、操作側 leader_agent の最初のパケットを待ってから追従開始。
#       パケット途絶 250ms で最終姿勢ホールド（脱力しない）・復帰時は 1s ランプで再追従。
# 前提: sudo bash scripts/canup.sh 済み（can_fol_r UP）。操作側は run_leader_agent_right.sh。
# 使い方: bash scripts/run_teleop_remote_right.sh
REPO="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$HOME/projects/lerobot/.venv/bin/lerobot-teleoperate"
POSE="$REPO/config/initial_pose_right.yaml"
POSE_ARGS=()
[ -f "$POSE" ] && POSE_ARGS+=(--robot.initial_pose_path="$POSE")
"$BIN" \
  --robot.type=openarm_follower --robot.port=can_fol_r --robot.side=right --robot.id=follower_right \
  --teleop.type=openarm_leader_remote --teleop.id=leader_right_remote \
  --teleop.zmq_port=5701 --teleop.echo_port=5711 \
  --fps=200 \
  "${POSE_ARGS[@]}"
