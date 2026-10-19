#!/usr/bin/env bash
# 別室テレオペ（操作側）: 右腕リーダーを 200Hz で読み、関節角を follower ホスト（Thor）へ ZMQ 送信する。
# 重力補償はこの PC 内で完結（ネットワークを跨がない）。設計書= toyota-body-orboh vault
# "openarm-remote-teleop-design"。停止は Ctrl-C（リーダーは脱力する＝先に腕を下ろす）。
# 前提: リーダー用 PEAK がこの PC に直挿しで sudo bash scripts/canup.sh 済み（can_lead_r が UP）。
# 使い方: THOR_IP=192.168.0.10 bash scripts/run_leader_agent_right.sh [scale]
SCALE="${1:-1.0}"
: "${THOR_IP:?THOR_IP=<followerホストのIP> を指定してください}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
URDF="$REPO/urdf/openarm_v10_bimanual.urdf"
PY="$HOME/projects/lerobot/.venv/bin/python"
# 開始姿勢: follower 側スクリプトと同じ YAML を使う（両腕が同じ整列先になることが重要）
# 開始姿勢セット（POSE_SET・未指定なら config/pose_sets/CURRENT）の right.yaml
source "$REPO/scripts/lib/pose_set.sh"
pose_set_load current || exit 1
POSE="$RPOSE"
POSE_ARGS=()
[ -f "$POSE" ] && POSE_ARGS+=(--teleop.initial_pose_path="$POSE")
"$PY" -m lerobot.teleoperators.openarm_leader_remote.leader_agent \
  --remote_host="$THOR_IP" --remote_port=5701 --echo_port=5711 \
  --arm=right --fps=200 \
  --teleop.port=can_lead_r --teleop.id=leader_right \
  --teleop.gravity_compensation=true \
  --teleop.gravity_urdf_path="$URDF" \
  --teleop.gravity_side=right \
  --teleop.gravity_scale="$SCALE" \
  --teleop.use_velocity_and_torque=false \
  "${POSE_ARGS[@]}"
# use_velocity_and_torque=false は設計 Phase 1（位置のみ）。ジッタ実測後に FF 有効化を検討（設計書 §4.3）
