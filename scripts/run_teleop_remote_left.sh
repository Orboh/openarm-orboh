#!/usr/bin/env bash
# 別室テレオペ（follower ホスト＝Thor 側）: ネットワーク越しのリーダー角度で左腕フォロワーを追従させる。
# 右腕版（run_teleop_remote_right.sh）の左腕版。ポートは 左=5702/echo 5712（右と共存＝両腕同時可・別ターミナルで並走）。
# 挙動: 操作側 leader_agent の最初のパケットを待ってから追従開始（30秒でタイムアウト終了→leader を先に起動）。
#       パケット途絶 250ms で最終姿勢ホールド（脱力しない）・復帰時は 1s ランプで再追従。
# 前提: sudo bash scripts/canup.sh 済み（can_fol_l UP）。操作側は run_leader_agent_left.sh。
# 使い方: bash scripts/run_teleop_remote_left.sh
REPO="$(cd "$(dirname "$0")/.." && pwd)"
BIN="$HOME/projects/lerobot/.venv/bin/lerobot-teleoperate"
POSE="$REPO/config/initial_pose_left.yaml"
POSE_ARGS=()
[ -f "$POSE" ] && POSE_ARGS+=(--robot.initial_pose_path="$POSE")
"$BIN" \
  --robot.type=openarm_follower --robot.port=can_fol_l --robot.side=left --robot.id=follower_left \
  --teleop.type=openarm_leader_remote --teleop.id=leader_left_remote \
  --teleop.zmq_port=5702 --teleop.echo_port=5712 \
  --fps=200 \
  "${POSE_ARGS[@]}"
